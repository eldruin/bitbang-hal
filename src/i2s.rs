//! Inter-IC Sound
//!
//! This implementation consumes the following hardware resources:
//! - Periodic timer to mark clock cycles
//! - Output GPIO pin for clock signal (BCLK)
//! - Output GPIO pin for data transmission (SD)
//! - Output GPIO pin for word (channel) selection (WS)
//!
//! The timer must be configured to twice the desired communication frequency.
//!
//! Both standard I2S and left-justified modes are supported.
//!

use embedded_hal::blocking::i2s;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::{CountDown, Periodic};
use nb::block;

/// I2S errors
#[derive(Debug)]
pub enum Error<EP> {
    /// GPIO error
    Pin(EP),
}

/// I2S mode
#[derive(Debug)]
pub enum Mode {
    /// Standard I2S
    I2s,
    /// Left-justified
    LeftJustified,
}

/// Bit banging I2S device
pub struct I2s<SCK, WS, SD, TIMER> {
    mode: Mode,
    sck: SCK,
    ws: WS,
    sd: SD,
    timer: TIMER,
}

impl<SCK, WS, SD, TIMER> I2s<SCK, WS, SD, TIMER> {
    /// Create new instance
    pub fn new(mode: Mode, sd: SD, ws: WS, sck: SCK, timer: TIMER) -> Self {
        I2s {
            mode,
            sck,
            ws,
            sd,
            timer,
        }
    }
}

macro_rules! impl_i2s_write {
    ($word_ty:ty, $raw_ty:ty, $bit_count:expr) => {
        impl<SCK, WS, SD, TIMER, EP> i2s::Write<$word_ty> for I2s<SCK, WS, SD, TIMER>
        where
            SCK: OutputPin<Error = EP>,
            WS: OutputPin<Error = EP>,
            SD: OutputPin<Error = EP>,
            TIMER: CountDown + Periodic,
        {
            type Error = Error<EP>;

            fn try_write<'w>(
                &mut self,
                left_words: &'w [$word_ty],
                right_words: &'w [$word_ty],
            ) -> Result<(), Self::Error> {
                self.set_ws_low()?;
                for (left_word, right_word) in left_words.iter().zip(right_words.iter()) {
                    self.try_write_words(
                        *left_word as $raw_ty,
                        *right_word as $raw_ty,
                        $bit_count,
                    )?;
                }
                Ok(())
            }
        }

        impl<SCK, WS, SD, TIMER, EP> i2s::WriteIter<$word_ty> for I2s<SCK, WS, SD, TIMER>
        where
            SCK: OutputPin<Error = EP>,
            WS: OutputPin<Error = EP>,
            SD: OutputPin<Error = EP>,
            TIMER: CountDown + Periodic,
        {
            type Error = Error<EP>;

            fn try_write_iter<LW, RW>(
                &mut self,
                left_words: LW,
                right_words: RW,
            ) -> Result<(), Self::Error>
            where
                LW: IntoIterator<Item = $word_ty>,
                RW: IntoIterator<Item = $word_ty>,
            {
                self.set_ws_low()?;
                for (left_word, right_word) in left_words.into_iter().zip(right_words.into_iter()) {
                    self.try_write_words(left_word as $raw_ty, right_word as $raw_ty, $bit_count)?;
                }
                Ok(())
            }
        }
    };
}
impl_i2s_write!(i16, u16, 16);
impl_i2s_write!(i32, u32, 32);

impl<SCK, WS, SD, TIMER, EP> I2s<SCK, WS, SD, TIMER>
where
    SCK: OutputPin<Error = EP>,
    WS: OutputPin<Error = EP>,
    SD: OutputPin<Error = EP>,
    TIMER: CountDown + Periodic,
{
    fn try_write_words<W>(
        &mut self,
        left_word: W,
        right_word: W,
        bit_count: u8,
    ) -> Result<(), Error<EP>>
    where
        W: core::ops::Shr<u8, Output = W>
            + core::ops::BitAnd<Output = W>
            + PartialEq
            + From<u8>
            + Copy,
    {
        match self.mode {
            Mode::I2s => {
                // The very first time this will be missing one bit if WS was set to high.
                // However, we cannot know about the previous call or the previous pin status.
                for offset in 0..(bit_count - 1) {
                    let bit = ((left_word >> (bit_count - (offset + 1))) & 1.into()) != 0.into();
                    self.try_write_bit(bit)?;
                }
                self.set_ws_high()?;
                self.try_write_bit((left_word & 1.into()) != 0.into())?; // last left bit

                for offset in 0..(bit_count - 1) {
                    let bit = ((right_word >> (bit_count - (offset + 1))) & 1.into()) != 0.into();
                    self.try_write_bit(bit)?;
                }
                self.set_ws_low()?;
                self.try_write_bit((right_word & 1.into()) != 0.into())?; // last right bit
            }
            Mode::LeftJustified => {
                for offset in 0..bit_count {
                    let bit = ((left_word >> (bit_count - (offset + 1))) & 1.into()) != 0.into();
                    self.try_write_bit(bit)?;
                }
                self.set_ws_high()?;

                for offset in 0..bit_count {
                    let bit = ((right_word >> (bit_count - (offset + 1))) & 1.into()) != 0.into();
                    self.try_write_bit(bit)?;
                }
                self.set_ws_low()?;
            }
        }
        Ok(())
    }

    fn try_write_bit(&mut self, bit: bool) -> Result<(), Error<EP>> {
        if bit {
            self.sd.set_high().map_err(Error::Pin)?;
        } else {
            self.sd.set_low().map_err(Error::Pin)?;
        }

        self.wait_clk();
        self.sck.set_high().map_err(Error::Pin)?; // data is sampled on the receiver
        self.wait_clk();
        self.sck.set_low().map_err(Error::Pin)
    }

    #[inline]
    fn wait_clk(&mut self) {
        block!(self.timer.wait()).unwrap()
    }

    #[inline]
    fn set_ws_high(&mut self) -> Result<(), Error<EP>> {
        self.ws.set_high().map_err(Error::Pin)
    }

    #[inline]
    fn set_ws_low(&mut self) -> Result<(), Error<EP>> {
        self.ws.set_low().map_err(Error::Pin)
    }
}
