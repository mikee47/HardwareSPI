TODO
====

-  Implement 64-bit addressing. Hardware supports this using ``wr_status`` register for high word.
-  Implement slave modes
-  Custom chip selects

Custom chip selects
-------------------

Multiplexed CS output
   Actual CS is still hardware, but re-routed for each device. Supported in any PinSet.
   Callback required to set this when request is executed.
   Consider generic callback with flag to indicate state, so it can be called at end of request as well.
   Note that this is not the same as a request completion callback as that is per-request.
   This new callback would be for the Device: must be an interrupt callback.

Manual CS control
   Only supported for PinSet::normal.
   Callback required at begin and end of each transaction.
   Provide an additional setting for the maximum transaction length so that requests can be split
   as usual or sent as a single transaction.
