# R4.2 Watermark Finalization Proof

## Design Principle
An interval `k` (from `t_i` to `t_j`) can only be finalized when no future out-of-order LIO sample could possibly influence the interpolation for `t_i` or `t_j`.
Because the maximum interpolation gap is `max_interpolation_gap_ns`, a sample up to `t_j + max_interpolation_gap_ns` could potentially serve as the right bracket for `t_j`. 
Therefore, finalization must wait until the event-time watermark clears `t_j + max_interpolation_gap_ns`.

## Code Evidence
In `shadow_timeline.cpp`, `tryInsertLioFactors()`:

```cpp
// Event-time finalization with influence horizon constraint:
if (watermark_ns_ < t_j + config_.max_interpolation_gap_ns)
{
    break;
}
```

This prevents any interval from being processed (and subsequently finalized) if the event watermark has not yet passed the influence horizon, completely solving the early-finalization bug.
Tests like `NotFinalizableAtAnchorEndOnly` and `FinalizableAfterInterpolationInfluenceHorizon` rigorously assert this behavior.
