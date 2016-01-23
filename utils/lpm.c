/**
 * @file low_power_mode.c
 * Manage low power modes.
 *
 * @author Stuart W. Baker
 * @date 20 April 2015
 */

#include "lpm.h"

/** Keeps track of current LPM mode.  This is a private variable and should
 * not be used directly by the application.
 */
LPM_Mode lpmMode;
