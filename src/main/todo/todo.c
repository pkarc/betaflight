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

#include "platform.h"

#ifdef USE_CMS_TODO_MENU

#include "fc/config.h"
#include "fc/dispatch.h"
#include "fc/runtime_config.h"

#include "pg/todo.h"

bool isTodoCompleted(void)
{

    for (unsigned i = 0; i < ARRAYLEN(todoConfigMutable()->tasks) ; i++) {
        if(!todoConfigMutable()->tasks[i].done) return false;
    }

   return true;
}

void resetTodo(void)
{
    for (unsigned i = 0; i < ARRAYLEN(todoConfigMutable()->tasks) ; i++) {
        todoConfigMutable()->tasks[i].done = false;
    }
}


#endif
