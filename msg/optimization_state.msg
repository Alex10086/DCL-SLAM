# One optimization round; participants and committed order are immutable.
Header header
uint64 round_id
int32 coordinator
# Immutable raw proposal, retained while READY advertises local PCM results.
uint64[] proposal_keys
int32[] participants
int32[] order
int8 state
bool committed
bool aborted
bool rotation_done
bool pose_done
# Canonical flattened (smaller key, larger key) separator edge pairs.
uint64[] separator_keys
