use super::types::SegmentKind;

const L1: (SegmentKind, i8) = (SegmentKind::Left, 1);
const R1: (SegmentKind, i8) = (SegmentKind::Right, 1);
const S1: (SegmentKind, i8) = (SegmentKind::Straight, 1);
const LN: (SegmentKind, i8) = (SegmentKind::Left, -1);
const RN: (SegmentKind, i8) = (SegmentKind::Right, -1);
const SN: (SegmentKind, i8) = (SegmentKind::Straight, -1);

pub(super) const PATHS: [&[(SegmentKind, i8)]; 48] = [
    &[L1, S1, L1],
    &[R1, S1, R1],
    &[LN, SN, LN],
    &[RN, SN, RN],
    &[L1, S1, R1],
    &[R1, S1, L1],
    &[LN, SN, RN],
    &[RN, SN, LN],
    &[L1, RN, L1],
    &[R1, LN, R1],
    &[LN, R1, LN],
    &[RN, L1, RN],
    &[L1, RN, LN],
    &[R1, LN, RN],
    &[LN, R1, L1],
    &[RN, L1, R1],
    &[L1, R1, LN],
    &[R1, L1, RN],
    &[LN, RN, L1],
    &[RN, LN, R1],
    &[L1, R1, LN, RN],
    &[R1, L1, RN, LN],
    &[LN, RN, L1, R1],
    &[RN, LN, R1, L1],
    &[L1, RN, LN, R1],
    &[R1, LN, RN, L1],
    &[LN, R1, L1, RN],
    &[RN, L1, R1, LN],
    &[L1, RN, SN, LN],
    &[R1, LN, SN, RN],
    &[LN, R1, S1, L1],
    &[RN, L1, S1, R1],
    &[L1, RN, SN, RN],
    &[R1, LN, SN, LN],
    &[LN, R1, S1, R1],
    &[RN, L1, S1, L1],
    &[L1, S1, R1, LN],
    &[R1, S1, L1, RN],
    &[LN, SN, RN, L1],
    &[RN, SN, LN, R1],
    &[L1, S1, L1, RN],
    &[R1, S1, R1, LN],
    &[LN, SN, LN, R1],
    &[RN, SN, RN, L1],
    &[L1, RN, SN, LN, R1],
    &[R1, LN, SN, RN, L1],
    &[LN, R1, S1, L1, RN],
    &[RN, L1, S1, R1, LN],
];

pub(super) const PATH_TYPE_INDICES: [(usize, usize, usize, usize); 12] = [
    (0, 1, 2, 3),
    (4, 5, 6, 7),
    (8, 9, 10, 11),
    (12, 13, 14, 15),
    (16, 17, 18, 19),
    (20, 21, 22, 23),
    (24, 25, 26, 27),
    (28, 29, 30, 31),
    (32, 33, 34, 35),
    (36, 37, 38, 39),
    (40, 41, 42, 43),
    (44, 45, 46, 47),
];
