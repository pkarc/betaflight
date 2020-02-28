/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS_TODO_MENU

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_todo.h"

#include "pg/pg.h"
#include "pg/todo.h"

#include "fc/config.h"

static todoTask_t tasks[];

static long cmsx_ToDo_onEnter(void)
{
    for (unsigned i = 0; i < ARRAYLEN(todoConfigMutable()->tasks) ; i++) {
        tasks[i] = todoConfigMutable()->tasks[i];
    }
    return 0;
}

static long cmsx_ToDo_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    for (unsigned i = 0; i < ARRAYLEN(todoConfigMutable()->tasks) ; i++) {
        tasks[i] = todoConfigMutable()->tasks[i];
    }

    return 0;
}

OSD_Entry buildOsdEntryForTask(int index){

   return (OSD_Entry){ &tasks[index].name, OME_Bool, NULL, &tasks[index].done, 0};

}

static const OSD_Entry cmsx_menuToDoEntries[] = {
    { "-- TODO --", OME_Label, NULL, NULL, 0},
    //Dinamically put the tasks
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 },
};


CMS_Menu cmsx_menuToDo = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUTODO",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_ToDo_onEnter,
    .onExit = cmsx_ToDo_onExit,
    .entries = cmsx_menuToDoEntries,
};

#endif
