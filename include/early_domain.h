/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */


/* Early domain services invoked in LK run in parallel after kernel
 * takes over. One page in memory is reserved to pass information between
 * LK and kernel. This page has a header to capture status, request and
 * cpumask described in structure early_domain_header. Early domain core
 * driver in kernel reads this header to decide the status of services
 * and takes necessary action. The rest of the page is intended to pass
 * service specific information. Offset for service specific area are
 * defined in macros, and its the service specific driver's responsiblity
 * to operate in their defined areas to pass service specific information.
 *
 *
 *
 ******************************************
 **		Header			  *
 **					  *
 ******************************************
 **					  *
 **		Early display		  *
 **					  *
 **					  *
 ******************************************
 **					  *
 **		Early camera		  *
 **					  *
 **					  *
 ******************************************
 **					  *
 **		Early audio		  *
 **					  *
 **					  *
 ******************************************
 */

#if EARLYDOMAIN_SUPPORT
#include <arch/defines.h>
#include <platform/iomap.h>

#define MAGIC_SIZE      8

struct early_domain_header
{
	char magic[MAGIC_SIZE];
	unsigned long long cpumask;
	unsigned long long status;
	volatile unsigned long long request;
};

#define EARLY_DOMAIN_MAGIC     "ERLYDOM"
#define NUM_SERVICES 		3
#define SECONDARY_CPU		1
#define SERVICE_SHARED_MEM_SIZE		((PAGE_SIZE)/(NUM_SERVICES))
#define EARLY_DISPLAY_SHM_START		(void *) (EARLY_DOMAIN_SHARED_MEM + sizeof(struct early_domain_header))
#define EARLY_CAMERA_SHM_START		(void *) (EARLY_DISPLAY_SHM_START + SERVICE_SHARED_MEM_SIZE)
#define EARLY_AUDIO_SHM_START		(void *) (EARLY_CAMERA_SHM_START + SERVICE_SHARED_MEM_SIZE)
#define EARLY_DOMAIN_CORE_DRIVER_DT_NODE	"/soc/early_domain"

enum service_id {
        EARLY_DOMAIN_CORE = 0,
        EARLY_DISPLAY = 1,
        EARLY_CAMERA = 2,
        EARLY_AUDIO = 3,
};

void clear_early_domain_status();
void set_early_service_active_bit(enum service_id sid);
void clear_early_service_active_bit(enum service_id sid);
bool get_early_service_shutdown_request(enum service_id sid);
bool get_early_service_active_bit(enum service_id sid);
void *get_service_shared_mem_start(enum service_id sid);
#endif /* EARLYDOMAIN_SUPPORT */
