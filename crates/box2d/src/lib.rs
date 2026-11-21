//! Port of the Box2D 2.0.x flavour used by the original game (see analysis/box2d-pretty.js, module
//! 23222). Everything is `f64` like the JS original so results can be compared bit-for-bit.

#![allow(clippy::needless_range_loop, clippy::neg_cmp_op_on_partial_ord, clippy::too_many_arguments)]

pub mod broadphase;
pub mod collision;
pub mod distance;
pub mod joints;
pub mod math;
pub mod settings;
pub mod shape;
pub mod world;
