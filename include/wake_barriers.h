#pragma once

// A pending command may span more than one kind of verified wake.
enum { BEFORE_DRIVER_WAKE = 1, BEFORE_BOARD_WAKE = 2 };
