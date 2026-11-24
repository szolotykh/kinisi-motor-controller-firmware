These unchanged 8,192-byte STM32 FRAM images came from the virtual-board public configuration-v1 fixtures at commit `1a1a39e`, produced by release firmware `3490977`. Factory is erased. Mecanum is the released 72-byte journal payload with nondefault geometry, encoders and 25 kHz PWM. The firmware test opens each input read-only and checks restored platform, stopped state and default encoder tracking preference.

| File | SHA-256 |
| --- | --- |
| factory.fram | 7d2c7ac4888bfd75cd5f56e8d61f69595121183afc81556c876732fd3782c62f |
| mecanum.fram | d492e8f032c427407fb232d459b65d0fa23d13ffebef2c7026764bc4de66231e |
