//------------------------------------------------------------
// File name: board_revision.h
// Description: Map the selected board build to the identity returned by INIT.
//------------------------------------------------------------
#pragma once

// Keep the existing V3 default; allow supported boards to select their build.
#if !defined(MP_V1) && !defined(MP_V2) && !defined(MP_V3)
#define MP_V3
#endif
#if (defined(MP_V1) + defined(MP_V2) + defined(MP_V3)) != 1
#error "Select exactly one supported board revision"
#endif

#define KINISI_BOARD_VERSION_MAJOR 0
#define KINISI_BOARD_VERSION_PATCH 0
#if defined(MP_V3)
#define KINISI_BOARD_VERSION_MINOR 3
#elif defined(MP_V2)
#define KINISI_BOARD_VERSION_MINOR 2
#else
#define KINISI_BOARD_VERSION_MINOR 1
#endif
