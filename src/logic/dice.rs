use embedded_layout_macros::ViewGroup;
use heapless::String;
use numtoa::NumToA;
use embedded_layout::prelude::*;
use embedded_graphics::prelude::*;
use core::fmt::Write;



#[derive(Copy, Clone)]
pub struct Slot {
    pub element_type: ElementKind,
    pub value: u16
}

#[derive(Copy, Clone)]
pub struct SlotArray {
    pub elements: [Slot; 10]
}

impl Slot {
pub fn slot_to_string(&self) -> String<1> {
let result =  match self.element_type {
            ElementKind::Blank => " ",
            ElementKind::D6 => {
                let mut buf : String<1> = String::new();
                write!(buf, "{:01}", self.value).unwrap();
                return buf;
            },
            ElementKind::GuardianDie => guardian_die_to_string(self.value),
            ElementKind::GrailCoin => grail_coin_to_string(self.value),
        };
        return String::from(result);
}
}

fn guardian_die_to_string(value: u16) -> &'static str {
    match value {
        1 => "N",
        2 => "E",
        3 => "S",
        4 => "W",
        5 => "X",
        6 => "-",
        _ => "?"
    }
}

fn grail_coin_to_string(value: u16) -> &'static str {
    match value {
        1 => "H",
        2 => "T",
        _ => "?"
    }
}


#[derive(Copy, Clone)]
pub enum ElementKind {
    Blank,
    D6,
    GuardianDie,
    GrailCoin
}