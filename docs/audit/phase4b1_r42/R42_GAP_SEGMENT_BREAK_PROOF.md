# R4.2 Gap Segment Break Proof

## Design Principle
When an IMU gap exceeds `max_imu_dt_ns`, the anchor status must become `INVALID_GAP`, propagating forward indefinitely. The timeline must completely cease returning valid predictions if the base state relies on a broken interval. No auto-recovery is allowed; it requires external reset.

## Code Evidence
In `shadow_timeline.cpp`, `feedImu()`:
```cpp
if (dt_ns > config_.max_imu_dt_ns)
{
    // ...
    anchor_status_[k + 1] = AnchorStatus::INVALID_GAP;
}
```

In `closeInterval(int k)`:
```cpp
if (anchor_status_[k] == AnchorStatus::INVALID_GAP)
{
    anchor_status_[k + 1] = AnchorStatus::INVALID_GAP;
    return;
}
```

In `propagateTo(stamp_ns)`:
```cpp
// Base anchor MUST be SOLVED
if (anchor_stamps_[i] <= stamp_ns && anchor_status_[i] == AnchorStatus::SOLVED)
```

By ensuring that `INVALID_GAP` propagates via `k+1`, any downstream anchor becomes invalid. `propagateTo` strictly searches for a `SOLVED` base anchor. Thus, broken segments are structurally unrecoverable until reset, fulfilling the strict anti-gap mandate.
