//---------------------------------------------------------------------------*
//                                                                           *
//                       File 'scheduletable_verif.h'                        *
//                 Generated by version GALGAS_BETA_VERSION                  *
//                      june 12th, 2009, at 14h44'54"                        *
//                                                                           *
//---------------------------------------------------------------------------*

//--- START OF USER ZONE 1


//--- END OF USER ZONE 1

#ifndef scheduletable_verif_DEFINED
#define scheduletable_verif_DEFINED

#include <string.h>

//---------------------------------------------------------------------------*

#include "galgas/C_GGS_Object.h"
#include "galgas/GGS_location.h"
#include "galgas/GGS_lbool.h"
#include "galgas/GGS_lchar.h"
#include "galgas/GGS_lstring.h"
#include "galgas/GGS_ldouble.h"
#include "galgas/GGS_luint.h"
#include "galgas/GGS_lsint.h"
#include "galgas/GGS_luint64.h"
#include "galgas/GGS_lsint64.h"
#include "galgas/GGS_stringset.h"
#include "galgas/GGS_binaryset.h"
#include "galgas/GGS_filewrapper.h"
#include "galgas/predefined_types.h"
#include "galgas/AC_galgas_class.h"
#include "galgas/AC_galgas_domain.h"
#include "galgas/AC_galgas_mapindex.h"
#include "galgas/AC_galgas_map.h"
#include "galgas/AC_galgas_listmap.h"
#include "galgas/AC_galgas_list.h"
#include "galgas/AC_galgas_sortedlist.h"

//---------------------------------------------------------------------------*

#include "galgas/C_Lexique.h"

// Include imported semantics
#include "goil_types_root.h"

//--- START OF USER ZONE 2


//--- END OF USER ZONE 2

//---------------------------------------------------------------------------*
//                                                                           *
//                          Class Predeclarations                            *
//                                                                           *
//---------------------------------------------------------------------------*


//---------------------------------------------------------------------------*
//                                                                           *
//                  Routine 'schedule_tables_well_formed'                    *
//                                                                           *
//---------------------------------------------------------------------------*

void routine_schedule_tables_well_formed (C_Compiler &,
                                const GGS_scheduletable_map  ,
                                const GGS_root_obj   COMMA_LOCATION_ARGS) ;

//---------------------------------------------------------------------------*
//                                                                           *
//         Routine 'schedule_tables_counters_exist_and_compatible'           *
//                                                                           *
//---------------------------------------------------------------------------*

void routine_schedule_tables_counters_exist_and_compatible (C_Compiler &,
                                const GGS_scheduletable_map  ,
                                const GGS_counter_map   COMMA_LOCATION_ARGS) ;

//---------------------------------------------------------------------------*
//                                                                           *
//             Routine 'schedule_tables_tasks_and_events_exist'              *
//                                                                           *
//---------------------------------------------------------------------------*

void routine_schedule_tables_tasks_and_events_exist (C_Compiler &,
                                const GGS_root_obj   COMMA_LOCATION_ARGS) ;

//---------------------------------------------------------------------------*

#endif