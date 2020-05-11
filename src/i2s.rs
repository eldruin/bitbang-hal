//! I2S - Inter-IC Sound

use embedded_hal::blocking::i2s;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::{CountDown, Periodic};
use nb::block;

#[derive(Debug)]
pub enum Error<EP> {
    Pin(EP),
}

#[derive(Debug)]
pub enum Mode {
    I2s,
    LeftJustified,
}

pub struct I2s<SCK, WS, SD, TIMER> {
    mode: Mode,
    sck: SCK,
    ws: WS,
    sd: SD,
    timer: TIMER,
}

impl<SCK, WS, SD, TIMER> I2s<SCK, WS, SD, TIMER> {
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
                let mut left = OutBitStream::new(left_words.iter());
                let mut right = OutBitStream::new(right_words.iter());
                match self.mode {
                    Mode::I2s => {
                        self.set_ws_low()?;
                        // The very first time this will be missing one bit if WS was set to high.
                        // However, we cannot know about the previous call or the previous pin status.
                        loop {
                            for _ in 0..($bit_count - 1) {
                                match left.next() {
                                    None => return Ok(()),
                                    Some(bit) => self.try_write_bit(bit)?,
                                }
                            }
                            self.set_ws_high()?;
                            self.try_write_bit(left.next().unwrap_or(false))?; // last left bit

                            for _ in 0..($bit_count - 1) {
                                self.try_write_bit(right.next().unwrap_or(false))?;
                            }
                            self.set_ws_low()?;
                            self.try_write_bit(right.next().unwrap_or(false))?; // last right bit
                        }
                    }
                    Mode::LeftJustified => loop {
                        self.set_ws_low()?;
                        for _ in 0..$bit_count {
                            match left.next() {
                                None => return Ok(()),
                                Some(bit) => self.try_write_bit(bit)?,
                            }
                        }

                        self.set_ws_high()?;
                        for _ in 0..$bit_count {
                            self.try_write_bit(right.next().unwrap_or(false))?;
                        }
                    },
                }
            }
        }
    };
}
impl_i2s_write!(i8, u8, 8);
impl_i2s_write!(i16, u16, 16);
impl_i2s_write!(i32, u32, 32);

impl<SCK, WS, SD, TIMER, EP> I2s<SCK, WS, SD, TIMER>
where
    SCK: OutputPin<Error = EP>,
    WS: OutputPin<Error = EP>,
    SD: OutputPin<Error = EP>,
    TIMER: CountDown + Periodic,
{
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
    fn wait_clk(&mut self) {
        block!(self.timer.wait()).unwrap()
    }
    fn set_ws_high(&mut self) -> Result<(), Error<EP>> {
        self.ws.set_high().map_err(Error::Pin)
    }
    fn set_ws_low(&mut self) -> Result<(), Error<EP>> {
        self.ws.set_low().map_err(Error::Pin)
    }
}

#[derive(Debug)]
struct OutBitStream<WI, WU> {
    data: WI,
    word_offset: usize,
    current_word: WU,
    bit_offset: usize,
}

impl<WI, WU> OutBitStream<WI, WU>
where
    WU: From<u8>,
{
    fn new(data: WI) -> Self {
        OutBitStream {
            data,
            word_offset: 0,
            current_word: 0.into(),
            bit_offset: 0,
        }
    }
}

macro_rules! impl_iterator_obs {
    ($wi:ty, $wu:ty, $word_bit_count:expr) => {
        impl<'a, WI> Iterator for OutBitStream<WI, $wu>
        where
            WI: Iterator<Item = &'a $wi>,
        {
            type Item = bool;

            #[inline]
            fn next(&mut self) -> Option<Self::Item> {
                if self.bit_offset == $word_bit_count {
                    self.bit_offset = 0;
                }
                if self.bit_offset == 0 {
                    match self.data.next() {
                        Some(value) => self.current_word = *value as $wu,
                        None => return None,
                    }
                }
                self.bit_offset += 1;
                // MSB first
                Some(((self.current_word >> ($word_bit_count - self.bit_offset)) & 1) != 0)
            }
        }
    };
}

impl_iterator_obs!(i8, u8, 8);
impl_iterator_obs!(i16, u16, 16);
impl_iterator_obs!(i32, u32, 32);

#[cfg(test)]
mod tests {
    // Can be run with `cargo test --target=x86_64-unknown-linux-gnu --lib`
    use super::*;

    #[test]
    fn can_stream_empty() {
        let mut obs: OutBitStream<core::slice::Iter<i8>, u8> = OutBitStream::new([].iter());
        assert_eq!(obs.next(), None);
    }
    #[test]
    fn can_stream_u8() {
        let mut obs = OutBitStream::new([0xAB_u8 as i8, 0xCD_u8 as i8].iter());
        for i in 0..8 {
            assert_eq!(obs.next(), Some((0xAB & (1 << (7 - i))) != 0));
        }
        for i in 0..8 {
            assert_eq!(obs.next(), Some((0xCD & (1 << (7 - i))) != 0));
        }

        assert_eq!(obs.next(), None);
    }

    #[test]
    fn can_stream_u16() {
        let mut obs = OutBitStream::new([0xABCD_u16 as i16, 0xEF01_u16 as i16].iter());
        for i in 0..16 {
            assert_eq!(obs.next(), Some((0xABCD & (1 << (15 - i))) != 0));
        }
        for i in 0..16 {
            assert_eq!(obs.next(), Some((0xEF01 & (1 << (15 - i))) != 0));
        }

        assert_eq!(obs.next(), None);
    }

    #[test]
    fn can_stream_u32() {
        let mut obs = OutBitStream::new([0xABCD_EF01_u32 as i32, 0x2345_6789_u32 as i32].iter());
        for i in 0..32 {
            assert_eq!(obs.next(), Some((0xABCD_EF01_u32 & (1 << (31 - i))) != 0));
        }
        for i in 0..32 {
            assert_eq!(obs.next(), Some((0x2345_6789_u32 & (1 << (31 - i))) != 0));
        }

        assert_eq!(obs.next(), None);
    }
}
