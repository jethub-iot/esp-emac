// SPDX-License-Identifier: GPL-2.0-or-later OR Apache-2.0
// Copyright (c) Viacheslav Bocharov <v@baodeep.com> and JetHome (r)

//! DMA descriptor management for the ESP32 EMAC.
//!
//! Provides TX and RX descriptor types, bit field constants, and a
//! generic circular ring buffer. The DMA engine uses chained descriptors:
//! each descriptor points to a data buffer and the next descriptor.

mod descriptor;
mod ring;

pub use descriptor::bits;
pub use descriptor::{RxDescriptor, TxDescriptor, VolatileCell};
pub use ring::DescriptorRing;
