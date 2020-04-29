pub use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3};

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{FullDuplex, Mode, Polarity};
use embedded_hal::timer::{CountDown, Periodic};
use nb::block;

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    NoData,
}

#[derive(Debug)]
pub enum BitOrder {
    MSBFirst,
    LSBFirst,
}

impl Default for BitOrder {
    /// Default bit order: MSB first
    fn default() -> Self {
        BitOrder::MSBFirst
    }
}

/// A Full-Duplex SPI implementation, takes 3 pins, and a timer running at 2x
/// the desired SPI frequency.
pub struct SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Timer: CountDown + Periodic,
{
    mode: Mode,
    miso: Miso,
    mosi: Mosi,
    sck: Sck,
    timer: Timer,
    read_val: Option<u8>,
    bit_order: BitOrder,
}

impl<Miso, Mosi, Sck, Timer, E> SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Timer: CountDown + Periodic,
{
    pub fn new(mode: Mode, miso: Miso, mosi: Mosi, sck: Sck, timer: Timer) -> Self {
        let mut spi = SPI {
            mode,
            miso,
            mosi,
            sck,
            timer,
            read_val: None,
            bit_order: BitOrder::default(),
        };

        match mode.polarity {
            Polarity::IdleLow => spi.sck.try_set_low(),
            Polarity::IdleHigh => spi.sck.try_set_high(),
        }
        .unwrap_or(());

        spi
    }

    pub fn set_bit_order(&mut self, order: BitOrder) {
        self.bit_order = order;
    }

    fn read_bit(&mut self) -> nb::Result<(), crate::spi::Error<E>> {
        if self.miso.try_is_high().map_err(Error::Bus)? {
            self.read_val = Some((self.read_val.unwrap_or(0) << 1) | 1);
            Ok(())
        } else {
            self.read_val = Some(self.read_val.unwrap_or(0) << 1);
            Ok(())
        }
    }
}

impl<Miso, Mosi, Sck, Timer, E> FullDuplex<u8> for SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Timer: CountDown + Periodic,
{
    type Error = crate::spi::Error<E>;

    fn try_read(&mut self) -> nb::Result<u8, Self::Error> {
        match self.read_val {
            Some(val) => Ok(val),
            None => Err(nb::Error::Other(crate::spi::Error::NoData)),
        }
    }

    fn try_send(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        for bit in 0..8 {
            let out_bit = match self.bit_order {
                BitOrder::MSBFirst => (byte >> (7 - bit)) & 0b1,
                BitOrder::LSBFirst => (byte >> bit) & 0b1,
            };

            if out_bit == 1 {
                self.mosi.try_set_high().map_err(Error::Bus)?;
            } else {
                self.mosi.try_set_low().map_err(Error::Bus)?;
            }

            match self.mode {
                MODE_0 => {
                    block!(self.timer.try_wait()).ok();
                    self.sck.try_set_high().map_err(Error::Bus)?;
                    self.read_bit()?;
                    block!(self.timer.try_wait()).ok();
                    self.sck.try_set_low().map_err(Error::Bus)?;
                }
                MODE_1 => {
                    self.sck.try_set_high().map_err(Error::Bus)?;
                    block!(self.timer.try_wait()).ok();
                    self.read_bit()?;
                    self.sck.try_set_low().map_err(Error::Bus)?;
                    block!(self.timer.try_wait()).ok();
                }
                MODE_2 => {
                    block!(self.timer.try_wait()).ok();
                    self.sck.try_set_low().map_err(Error::Bus)?;
                    self.read_bit()?;
                    block!(self.timer.try_wait()).ok();
                    self.sck.try_set_high().map_err(Error::Bus)?;
                }
                MODE_3 => {
                    self.sck.try_set_low().map_err(Error::Bus)?;
                    block!(self.timer.try_wait()).ok();
                    self.read_bit()?;
                    self.sck.try_set_high().map_err(Error::Bus)?;
                    block!(self.timer.try_wait()).ok();
                }
            }
        }

        Ok(())
    }
}

impl<Miso, Mosi, Sck, Timer, E> embedded_hal::blocking::spi::transfer::Default<u8>
    for SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Timer: CountDown + Periodic,
{
}

impl<Miso, Mosi, Sck, Timer, E> embedded_hal::blocking::spi::write::Default<u8>
    for SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Timer: CountDown + Periodic,
{
}
