/**
 * Request.cpp
 *
 * Copyright 2018 mikee47 <mike@sillyhouse.net>
 *
 * This file is part of the HardwareSPI Library
 *
 * This library is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, version 3 or later.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this library.
 * If not, see <https://www.gnu.org/licenses/>.
 *
 * 
 * @author: 11 December 2018 - mikee47 <mike@sillyhouse.net>
 *
 ****/

#include "include/HSPI/Request.h"
#include <esp_attr.h>

namespace HSPI
{
/**
 * @brief Support function for fast request re-queuing
 *
 * Append a request back onto the queue.
 *
 * Don't just put it at the front of the queue though as it will block any other requests.
 * So it goes at the end. BUT! we must preserve execution order for all requests for a given device,
 * so any other requests for the same device must go after it.
 *
 * The easiest way to do this is to build two queues, one off the current request (A) and the other containing all other requests (B).
 * We then append (A) to (B) and set the head to the start of (B).
 * 
 * This is kind of laborious but fast as it's just pointer manipulation.
 */
Request* IRAM_ATTR reQueueRequest(Request* head, Request* request)
{
	Request otherQueue; // This just gives us something to point to
	Request* otherTail = &otherQueue;
	Request* thisTail = request;

	auto cur = head;
	while(cur != nullptr) {
		if(cur->device == request->device) {
			thisTail->next = cur;
			thisTail = cur;
		} else {
			otherTail->next = cur;
			otherTail = cur;
		}
		cur = cur->next;
	}

	// Now merge the queues
	thisTail->next = nullptr;
	otherTail->next = request;

	return otherQueue.next;
}

} // namespace HSPI
