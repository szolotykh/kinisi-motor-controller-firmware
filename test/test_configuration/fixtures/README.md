These unchanged 8,192-byte STM32 FRAM images came from the virtual-board public configuration-v1 fixtures at commit `1a1a39e`, produced by release firmware `3490977`. Factory is erased. Mecanum is the released 72-byte journal payload with nondefault geometry, encoders and 25 kHz PWM. The firmware test opens each input read-only and checks restored platform, stopped state and default encoder tracking preference.

| File | SHA-256 |
| --- | --- |
| factory.fram | 7d2c7ac4888bfd75cd5f56e8d61f69595121183afc81556c876732fd3782c62f |
| mecanum.fram | d492e8f032c427407fb232d459b65d0fa23d13ffebef2c7026764bc4de66231e |

The two protocol 2.25 regression images were written by the production firmware
dispatch and journal in commit `ac4ccef34ca0be8da119dbaaf23e41c6ae22f386`, using
that commit's v0.4 native configuration harness. The current harness cold-boots
both 8192-byte images and reads the running geometry, directions and mount.

| File | Original harness mode and argument | SHA-256 |
| --- | --- | --- |
| `v25-mirrored-board-mount.fram` | `roller-save`, 2 (PCB underside mount) | `101402ff8d4180e506766f4eb8984ead1f832140d32c759c0579b1fcb184bfae` |
| `v25-omni-rear.fram` | `save`, 2 (legacy Omni) | `b0185da6f09dc231b67a8f9435f3ac381a5d3f8dc30de14198f63a9767a2bad1` |
