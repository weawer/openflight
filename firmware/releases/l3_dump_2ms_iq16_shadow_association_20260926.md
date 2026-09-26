# IWR6843 shadow-selector association correction

- Binary: `l3_dump_2ms_iq16_shadow_association_20260926.bin`
- SHA-256: `8930da7f7e66722acb3cd750eae37595802fa37dd7eb23a2866b2d50e0f4bf99`
- Reference profile: `config/iwr6843_l3dump_shadow_reference_7f2ms_128bin_iq16.cfg`

The first paired reference capture showed that stale velocity after a missed
candidate could move the proposed window away from a slower real target. This
image holds the last range and clears velocity on a miss. It also limits
association to four bins per 2 ms frame, which covers approximately 90 m/s at
the profile's range resolution.

The original full-range capture is retained as a regression vector. With the
corrected rule, its smooth 13-to-15-bin moving return remains inside every
proposed 12-bin window.
