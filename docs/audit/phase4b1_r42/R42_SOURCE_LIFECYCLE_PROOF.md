# R4.2 Anchor & Constraint Lifecycle Proof

## Anchor 0 Lifecycle

In `feedImu()`, the first anchor used to bypass standard progression and push `SOLVED` immediately. It has been corrected:
```cpp
if (anchor_stamps_.empty())
{
    // ...
    anchor_status_.push_back(AnchorStatus::GRAPH_INSERTED);
    // Add Gauge priors
}
```
This forces it to go through `flushOptimizer()`, correctly integrating it to `SOLVED` during the first state update.

## Enum Cleanup
Unused states `OPEN` and `IMU_COMPLETE` have been entirely removed from the `AnchorStatus` enum.

## Constraint Finalization Tracking

Constraint slot management is now cleanly split:
- `committed_slots_`: Tracks slots that have successfully been integrated into the graph (factor added). Used to return `isIntervalLioConstrained()` and prevent `REJECT_SLOT_OCCUPIED`.
- `finalized_slots_`: Tracks slots that have reached a terminal outcome (whether ACCEPTED, TOO_LATE, REJECT_INNOVATION, NO_BRACKET, CROSS_EPOCH). Used to advance the `next_lio_scan_k_` forward.

In `insertRelativeConstraint`:
```cpp
// On success:
committed_slots_.insert(slot);
finalized_slots_.insert(slot);
```
```cpp
// On rejection (e.g. innovation bound failed):
finalized_slots_.insert(slot);
return AcceptDecision::REJECT_INNOVATION_TRANS;
```

In `tryInsertLioFactors`:
```cpp
// explicit Cross-Epoch and No-Bracket finalization
if (dec_i == AcceptDecision::REJECT_CROSS_EPOCH || dec_j == AcceptDecision::REJECT_CROSS_EPOCH) {
    finalized_slots_.insert({0, k});
    continue;
}
```

This guarantees no infinite retries for fundamentally invalid data.
