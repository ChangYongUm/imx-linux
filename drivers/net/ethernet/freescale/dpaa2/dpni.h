/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2013-2016 Freescale Semiconductor Inc.
 * Copyright 2016, 2020, 2024 NXP
 */
#ifndef __FSL_DPNI_H
#define __FSL_DPNI_H

#include "dpkg.h"

struct fsl_mc_io;

/* Data Path Network Interface API
 * Contains initialization APIs and runtime control APIs for DPNI
 */

/** General DPNI macros */

/**
 * DPNI_MAX_TC - Maximum number of traffic classes
 */
#define DPNI_MAX_TC				8
/**
 * DPNI_MAX_DPBP - Maximum number of buffer pools per DPNI
 */
#define DPNI_MAX_DPBP				8

/**
 * DPNI_ALL_TCS - All traffic classes considered; see dpni_set_queue()
 */
#define DPNI_ALL_TCS				(u8)(-1)
/**
 * DPNI_ALL_TC_FLOWS - All flows within traffic class considered; see
 * dpni_set_queue()
 */
#define DPNI_ALL_TC_FLOWS			(u16)(-1)
/**
 * DPNI_NEW_FLOW_ID - Generate new flow ID; see dpni_set_queue()
 */
#define DPNI_NEW_FLOW_ID			(u16)(-1)

/**
 * DPNI_OPT_TX_FRM_RELEASE - Tx traffic is always released to a buffer pool on
 * transmit, there are no resources allocated to have the frames confirmed back
 * to the source after transmission.
 */
#define DPNI_OPT_TX_FRM_RELEASE			0x000001
/**
 * DPNI_OPT_NO_MAC_FILTER - Disables support for MAC address filtering for
 * addresses other than primary MAC address. This affects both unicast and
 * multicast. Promiscuous mode can still be enabled/disabled for both unicast
 * and multicast. If promiscuous mode is disabled, only traffic matching the
 * primary MAC address will be accepted.
 */
#define DPNI_OPT_NO_MAC_FILTER			0x000002
/**
 * DPNI_OPT_HAS_POLICING - Allocate policers for this DPNI. They can be used to
 * rate-limit traffic per traffic class (TC) basis.
 */
#define DPNI_OPT_HAS_POLICING			0x000004
/**
 * DPNI_OPT_SHARED_CONGESTION - Congestion can be managed in several ways,
 * allowing the buffer pool to deplete on ingress, taildrop on each queue or
 * use congestion groups for sets of queues. If set, it configures a single
 * congestion groups across all TCs.  If reset, a congestion group is allocated
 * for each TC. Only relevant if the DPNI has multiple traffic classes.
 */
#define DPNI_OPT_SHARED_CONGESTION		0x000008
/**
 * DPNI_OPT_HAS_KEY_MASKING - Enables TCAM for Flow Steering and QoS look-ups.
 * If not specified, all look-ups are exact match. Note that TCAM is not
 * available on LS1088 and its variants. Setting this bit on these SoCs will
 * trigger an error.
 */
#define DPNI_OPT_HAS_KEY_MASKING		0x000010
/**
 * DPNI_OPT_NO_FS - Disables the flow steering table.
 */
#define DPNI_OPT_NO_FS				0x000020
/**
 * DPNI_OPT_SHARED_FS - Flow steering table is shared between all traffic
 * classes
 */
#define DPNI_OPT_SHARED_FS			0x001000

int dpni_open(struct fsl_mc_io	*mc_io,
	      u32		cmd_flags,
	      int		dpni_id,
	      u16		*token);

int dpni_close(struct fsl_mc_io	*mc_io,
	       u32		cmd_flags,
	       u16		token);

#define DPNI_POOL_ASSOC_QPRI	0
#define DPNI_POOL_ASSOC_QDBIN	1

/**
 * struct dpni_pools_cfg - Structure representing buffer pools configuration
 * @num_dpbp: Number of DPBPs
 * @pool_options: Buffer assignment options.
 *	This field is a combination of DPNI_POOL_ASSOC_flags
 * @pools: Array of buffer pools parameters; The number of valid entries
 *	must match 'num_dpbp' value
 * @pools.dpbp_id: DPBP object ID
 * @pools.priority: Priority mask that indicates TC's used with this buffer.
 *	If set to 0x00 MC will assume value 0xff.
 * @pools.buffer_size: Buffer size
 * @pools.backup_pool: Backup pool
 */
struct dpni_pools_cfg {
	u8		num_dpbp;
	u8		pool_options;
	struct {
		int	dpbp_id;
		u8	priority_mask;
		u16	buffer_size;
		int	backup_pool;
	} pools[DPNI_MAX_DPBP];
};

int dpni_set_pools(struct fsl_mc_io		*mc_io,
		   u32				cmd_flags,
		   u16				token,
		   const struct dpni_pools_cfg	*cfg);

int dpni_enable(struct fsl_mc_io	*mc_io,
		u32			cmd_flags,
		u16			token);

int dpni_disable(struct fsl_mc_io	*mc_io,
		 u32			cmd_flags,
		 u16			token);

int dpni_is_enabled(struct fsl_mc_io	*mc_io,
		    u32			cmd_flags,
		    u16			token,
		    int			*en);

int dpni_reset(struct fsl_mc_io	*mc_io,
	       u32		cmd_flags,
	       u16		token);

/* DPNI IRQ Index and Events */

#define DPNI_IRQ_INDEX				0

/* DPNI_IRQ_EVENT_LINK_CHANGED - indicates a change in link state */
#define DPNI_IRQ_EVENT_LINK_CHANGED		0x00000001

/* DPNI_IRQ_EVENT_ENDPOINT_CHANGED - indicates a change in endpoint */
#define DPNI_IRQ_EVENT_ENDPOINT_CHANGED		0x00000002

int dpni_set_irq_enable(struct fsl_mc_io	*mc_io,
			u32			cmd_flags,
			u16			token,
			u8			irq_index,
			u8			en);

int dpni_get_irq_enable(struct fsl_mc_io	*mc_io,
			u32			cmd_flags,
			u16			token,
			u8			irq_index,
			u8			*en);

int dpni_set_irq_mask(struct fsl_mc_io	*mc_io,
		      u32		cmd_flags,
		      u16		token,
		      u8		irq_index,
		      u32		mask);

int dpni_get_irq_mask(struct fsl_mc_io	*mc_io,
		      u32		cmd_flags,
		      u16		token,
		      u8		irq_index,
		      u32		*mask);

int dpni_get_irq_status(struct fsl_mc_io	*mc_io,
			u32			cmd_flags,
			u16			token,
			u8			irq_index,
			u32			*status);

int dpni_clear_irq_status(struct fsl_mc_io	*mc_io,
			  u32			cmd_flags,
			  u16			token,
			  u8			irq_index,
			  u32			status);

/**
 * struct dpni_attr - Structure representing DPNI attributes
 * @options: Any combination of the following options:
 *		DPNI_OPT_TX_FRM_RELEASE
 *		DPNI_OPT_NO_MAC_FILTER
 *		DPNI_OPT_HAS_POLICING
 *		DPNI_OPT_SHARED_CONGESTION
 *		DPNI_OPT_HAS_KEY_MASKING
 *		DPNI_OPT_NO_FS
 * @num_queues: Number of Tx and Rx queues used for traffic distribution.
 * @num_tcs: Number of traffic classes (TCs), reserved for the DPNI.
 * @mac_filter_entries: Number of entries in the MAC address filtering table.
 * @vlan_filter_entries: Number of entries in the VLAN address filtering table.
 * @qos_entries: Number of entries in the QoS classification table.
 * @fs_entries: Number of entries in the flow steering table.
 * @qos_key_size: Size, in bytes, of the QoS look-up key. Defining a key larger
 *		than this when adding QoS entries will result in an error.
 * @fs_key_size: Size, in bytes, of the flow steering look-up key. Defining a
 *		key larger than this when composing the hash + FS key will
 *		result in an error.
 * @wriop_version: Version of WRIOP HW block. The 3 version values are stored
 *		on 6, 5, 5 bits respectively.
 */
struct dpni_attr {
	u32 options;
	u8 num_queues;
	u8 num_rx_tcs;
	u8 num_tx_tcs;
	u8 mac_filter_entries;
	u8 vlan_filter_entries;
	u8 qos_entries;
	u16 fs_entries;
	u8 qos_key_size;
	u8 fs_key_size;
	u16 wriop_version;
};

int dpni_get_attributes(struct fsl_mc_io	*mc_io,
			u32			cmd_flags,
			u16			token,
			struct dpni_attr	*attr,
			u16 cmdid);

/* DPNI errors */

/**
 * DPNI_ERROR_EOFHE - Extract out of frame header error
 */
#define DPNI_ERROR_EOFHE	0x00020000
/**
 * DPNI_ERROR_FLE - Frame length error
 */
#define DPNI_ERROR_FLE		0x00002000
/**
 * DPNI_ERROR_FPE - Frame physical error
 */
#define DPNI_ERROR_FPE		0x00001000
/**
 * DPNI_ERROR_PHE - Parsing header error
 */
#define DPNI_ERROR_PHE		0x00000020
/**
 * DPNI_ERROR_L3CE - Parser L3 checksum error
 */
#define DPNI_ERROR_L3CE		0x00000004
/**
 * DPNI_ERROR_L4CE - Parser L3 checksum error
 */
#define DPNI_ERROR_L4CE		0x00000001

/**
 * enum dpni_error_action - Defines DPNI behavior for errors
 * @DPNI_ERROR_ACTION_DISCARD: Discard the frame
 * @DPNI_ERROR_ACTION_CONTINUE: Continue with the normal flow
 * @DPNI_ERROR_ACTION_SEND_TO_ERROR_QUEUE: Send the frame to the error queue
 */
enum dpni_error_action {
	DPNI_ERROR_ACTION_DISCARD = 0,
	DPNI_ERROR_ACTION_CONTINUE = 1,
	DPNI_ERROR_ACTION_SEND_TO_ERROR_QUEUE = 2
};

/**
 * struct dpni_error_cfg - Structure representing DPNI errors treatment
 * @errors: Errors mask; use 'DPNI_ERROR__<X>
 * @error_action: The desired action for the errors mask
 * @set_frame_annotation: Set to '1' to mark the errors in frame annotation
 *		status (FAS); relevant only for the non-discard action
 */
struct dpni_error_cfg {
	u32			errors;
	enum dpni_error_action	error_action;
	int			set_frame_annotation;
};

int dpni_set_errors_behavior(struct fsl_mc_io		*mc_io,
			     u32			cmd_flags,
			     u16			token,
			     struct dpni_error_cfg	*cfg);

/* DPNI buffer layout modification options */

/**
 * DPNI_BUF_LAYOUT_OPT_TIMESTAMP - Select to modify the time-stamp setting
 */
#define DPNI_BUF_LAYOUT_OPT_TIMESTAMP		0x00000001
/**
 * DPNI_BUF_LAYOUT_OPT_PARSER_RESULT - Select to modify the parser-result
 * setting; not applicable for Tx
 */
#define DPNI_BUF_LAYOUT_OPT_PARSER_RESULT	0x00000002
/**
 * DPNI_BUF_LAYOUT_OPT_FRAME_STATUS - Select to modify the frame-status setting
 */
#define DPNI_BUF_LAYOUT_OPT_FRAME_STATUS	0x00000004
/**
 * DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE - Select to modify the private-data-size setting
 */
#define DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE	0x00000008
/**
 * DPNI_BUF_LAYOUT_OPT_DATA_ALIGN - Select to modify the data-alignment setting
 */
#define DPNI_BUF_LAYOUT_OPT_DATA_ALIGN		0x00000010
/**
 * DPNI_BUF_LAYOUT_OPT_DATA_HEAD_ROOM - Select to modify the data-head-room setting
 */
#define DPNI_BUF_LAYOUT_OPT_DATA_HEAD_ROOM	0x00000020
/**
 * DPNI_BUF_LAYOUT_OPT_DATA_TAIL_ROOM - Select to modify the data-tail-room setting
 */
#define DPNI_BUF_LAYOUT_OPT_DATA_TAIL_ROOM	0x00000040

/**
 * struct dpni_buffer_layout - Structure representing DPNI buffer layout
 * @options: Flags representing the suggested modifications to the buffer
 *		layout; Use any combination of 'DPNI_BUF_LAYOUT_OPT_<X>' flags
 * @pass_timestamp: Pass timestamp value
 * @pass_parser_result: Pass parser results
 * @pass_frame_status: Pass frame status
 * @private_data_size: Size kept for private data (in bytes)
 * @data_align: Data alignment
 * @data_head_room: Data head room
 * @data_tail_room: Data tail room
 */
struct dpni_buffer_layout {
	u32	options;
	int	pass_timestamp;
	int	pass_parser_result;
	int	pass_frame_status;
	u16	private_data_size;
	u16	data_align;
	u16	data_head_room;
	u16	data_tail_room;
};

/**
 * enum dpni_queue_type - Identifies a type of queue targeted by the command
 * @DPNI_QUEUE_RX: Rx queue
 * @DPNI_QUEUE_TX: Tx queue
 * @DPNI_QUEUE_TX_CONFIRM: Tx confirmation queue
 * @DPNI_QUEUE_RX_ERR: Rx error queue
 */
enum dpni_queue_type {
	DPNI_QUEUE_RX,
	DPNI_QUEUE_TX,
	DPNI_QUEUE_TX_CONFIRM,
	DPNI_QUEUE_RX_ERR,
};

int dpni_get_buffer_layout(struct fsl_mc_io		*mc_io,
			   u32				cmd_flags,
			   u16				token,
			   enum dpni_queue_type		qtype,
			   struct dpni_buffer_layout	*layout);

int dpni_set_buffer_layout(struct fsl_mc_io		   *mc_io,
			   u32				   cmd_flags,
			   u16				   token,
			   enum dpni_queue_type		   qtype,
			   const struct dpni_buffer_layout *layout);

/**
 * enum dpni_offload - Identifies a type of offload targeted by the command
 * @DPNI_OFF_RX_L3_CSUM: Rx L3 checksum validation
 * @DPNI_OFF_RX_L4_CSUM: Rx L4 checksum validation
 * @DPNI_OFF_TX_L3_CSUM: Tx L3 checksum generation
 * @DPNI_OFF_TX_L4_CSUM: Tx L4 checksum generation
 */
enum dpni_offload {
	DPNI_OFF_RX_L3_CSUM,
	DPNI_OFF_RX_L4_CSUM,
	DPNI_OFF_TX_L3_CSUM,
	DPNI_OFF_TX_L4_CSUM,
};

int dpni_set_offload(struct fsl_mc_io	*mc_io,
		     u32		cmd_flags,
		     u16		token,
		     enum dpni_offload	type,
		     u32		config);

int dpni_get_offload(struct fsl_mc_io	*mc_io,
		     u32		cmd_flags,
		     u16		token,
		     enum dpni_offload	type,
		     u32		*config);

int dpni_get_qdid(struct fsl_mc_io	*mc_io,
		  u32			cmd_flags,
		  u16			token,
		  enum dpni_queue_type	qtype,
		  u16			*qdid);

int dpni_get_tx_data_offset(struct fsl_mc_io	*mc_io,
			    u32			cmd_flags,
			    u16			token,
			    u16			*data_offset);

#define DPNI_STATISTICS_CNT		7
#define DPNI_STATISTICS_32_CNT		14

/**
 * union dpni_statistics - Union describing the DPNI statistics
 * @page_0: Page_0 statistics structure
 * @page_0.ingress_all_frames: Ingress frame count
 * @page_0.ingress_all_bytes: Ingress byte count
 * @page_0.ingress_multicast_frames: Ingress multicast frame count
 * @page_0.ingress_multicast_bytes: Ingress multicast byte count
 * @page_0.ingress_broadcast_frames: Ingress broadcast frame count
 * @page_0.ingress_broadcast_bytes: Ingress broadcast byte count
 * @page_1: Page_1 statistics structure
 * @page_1.egress_all_frames: Egress frame count
 * @page_1.egress_all_bytes: Egress byte count
 * @page_1.egress_multicast_frames: Egress multicast frame count
 * @page_1.egress_multicast_bytes: Egress multicast byte count
 * @page_1.egress_broadcast_frames: Egress broadcast frame count
 * @page_1.egress_broadcast_bytes: Egress broadcast byte count
 * @page_2: Page_2 statistics structure
 * @page_2.ingress_filtered_frames: Ingress filtered frame count
 * @page_2.ingress_discarded_frames: Ingress discarded frame count
 * @page_2.ingress_nobuffer_discards: Ingress discarded frame count due to
 *	lack of buffers
 * @page_2.egress_discarded_frames: Egress discarded frame count
 * @page_2.egress_confirmed_frames: Egress confirmed frame count
 * @page_3: Page_3 statistics structure
 * @page_3.egress_dequeue_bytes: Cumulative count of the number of bytes
 *	dequeued from egress FQs
 * @page_3.egress_dequeue_frames: Cumulative count of the number of frames
 *	dequeued from egress FQs
 * @page_3.egress_reject_bytes: Cumulative count of the number of bytes in
 *	egress frames whose enqueue was rejected
 * @page_3.egress_reject_frames: Cumulative count of the number of egress
 *	frames whose enqueue was rejected
 * @page_4: Page_4 statistics structure: congestion points
 * @page_4.cgr_reject_frames: number of rejected frames due to congestion point
 * @page_4.cgr_reject_bytes: number of rejected bytes due to congestion point
 * @page_5: Page_5 statistics structure: policer
 * @page_5.policer_cnt_red: NUmber of red colored frames
 * @page_5.policer_cnt_yellow: number of yellow colored frames
 * @page_5.policer_cnt_green: number of green colored frames
 * @page_5.policer_cnt_re_red: number of recolored red frames
 * @page_5.policer_cnt_re_yellow: number of recolored yellow frames
 * @page_6: Page_6 statistics structure
 * @page_6.tx_pending_frames: total number of frames pending in egress FQs
 * @raw: raw statistics structure, used to index counters
 */
union dpni_statistics {
	struct {
		u64 ingress_all_frames;
		u64 ingress_all_bytes;
		u64 ingress_multicast_frames;
		u64 ingress_multicast_bytes;
		u64 ingress_broadcast_frames;
		u64 ingress_broadcast_bytes;
	} page_0;
	struct {
		u64 egress_all_frames;
		u64 egress_all_bytes;
		u64 egress_multicast_frames;
		u64 egress_multicast_bytes;
		u64 egress_broadcast_frames;
		u64 egress_broadcast_bytes;
	} page_1;
	struct {
		u64 ingress_filtered_frames;
		u64 ingress_discarded_frames;
		u64 ingress_nobuffer_discards;
		u64 egress_discarded_frames;
		u64 egress_confirmed_frames;
	} page_2;
	struct {
		u64 egress_dequeue_bytes;
		u64 egress_dequeue_frames;
		u64 egress_reject_bytes;
		u64 egress_reject_frames;
	} page_3;
	struct {
		u64 cgr_reject_frames;
		u64 cgr_reject_bytes;
	} page_4;
	struct {
		u64 policer_cnt_red;
		u64 policer_cnt_yellow;
		u64 policer_cnt_green;
		u64 policer_cnt_re_red;
		u64 policer_cnt_re_yellow;
	} page_5;
	struct {
		u64 tx_pending_frames;
	} page_6;
	struct {
		u64 counter[DPNI_STATISTICS_CNT];
	} raw;
};

int dpni_get_statistics(struct fsl_mc_io	*mc_io,
			u32			cmd_flags,
			u16			token,
			u8			page,
			u8			param,
			union dpni_statistics	*stat);

int dpni_reset_statistics(struct fsl_mc_io	*mc_io,
			  u32			cmd_flags,
			  u16			token);

#define DPNI_LINK_OPT_AUTONEG		0x0000000000000001ULL
#define DPNI_LINK_OPT_HALF_DUPLEX	0x0000000000000002ULL
#define DPNI_LINK_OPT_PAUSE		0x0000000000000004ULL
#define DPNI_LINK_OPT_ASYM_PAUSE	0x0000000000000008ULL
#define DPNI_LINK_OPT_PFC_PAUSE		0x0000000000000010ULL

/**
 * struct dpni_link_cfg - Structure representing DPNI link configuration
 * @rate: Rate
 * @options: Mask of available options; use 'DPNI_LINK_OPT_<X>' values
 */
struct dpni_link_cfg {
	u32 rate;
	u64 options;
};

int dpni_set_link_cfg(struct fsl_mc_io			*mc_io,
		      u32				cmd_flags,
		      u16				token,
		      const struct dpni_link_cfg	*cfg);

int dpni_get_link_cfg(struct fsl_mc_io			*mc_io,
		      u32				cmd_flags,
		      u16				token,
		      struct dpni_link_cfg		*cfg);

/**
 * struct dpni_link_state - Structure representing DPNI link state
 * @rate: Rate
 * @options: Mask of available options; use 'DPNI_LINK_OPT_<X>' values
 * @up: Link state; '0' for down, '1' for up
 */
struct dpni_link_state {
	u32	rate;
	u64	options;
	int	up;
};

int dpni_get_link_state(struct fsl_mc_io	*mc_io,
			u32			cmd_flags,
			u16			token,
			struct dpni_link_state	*state);

int dpni_set_max_frame_length(struct fsl_mc_io	*mc_io,
			      u32		cmd_flags,
			      u16		token,
			      u16		max_frame_length);

int dpni_get_max_frame_length(struct fsl_mc_io	*mc_io,
			      u32		cmd_flags,
			      u16		token,
			      u16		*max_frame_length);

int dpni_set_multicast_promisc(struct fsl_mc_io *mc_io,
			       u32		cmd_flags,
			       u16		token,
			       int		en);

int dpni_get_multicast_promisc(struct fsl_mc_io *mc_io,
			       u32		cmd_flags,
			       u16		token,
			       int		*en);

int dpni_set_unicast_promisc(struct fsl_mc_io	*mc_io,
			     u32		cmd_flags,
			     u16		token,
			     int		en);

int dpni_get_unicast_promisc(struct fsl_mc_io	*mc_io,
			     u32		cmd_flags,
			     u16		token,
			     int		*en);

int dpni_set_primary_mac_addr(struct fsl_mc_io *mc_io,
			      u32		cmd_flags,
			      u16		token,
			      const u8		mac_addr[6]);

int dpni_get_primary_mac_addr(struct fsl_mc_io	*mc_io,
			      u32		cmd_flags,
			      u16		token,
			      u8		mac_addr[6]);

int dpni_get_port_mac_addr(struct fsl_mc_io	*mc_io,
			   u32			cm_flags,
			   u16			token,
			   u8			mac_addr[6]);

int dpni_add_mac_addr(struct fsl_mc_io	*mc_io,
		      u32		cmd_flags,
		      u16		token,
		      const u8		mac_addr[6]);

int dpni_remove_mac_addr(struct fsl_mc_io	*mc_io,
			 u32			cmd_flags,
			 u16			token,
			 const u8		mac_addr[6]);

int dpni_clear_mac_filters(struct fsl_mc_io	*mc_io,
			   u32			cmd_flags,
			   u16			token,
			   int			unicast,
			   int			multicast);

/**
 * enum dpni_dist_mode - DPNI distribution mode
 * @DPNI_DIST_MODE_NONE: No distribution
 * @DPNI_DIST_MODE_HASH: Use hash distribution; only relevant if
 *		the 'DPNI_OPT_DIST_HASH' option was set at DPNI creation
 * @DPNI_DIST_MODE_FS:  Use explicit flow steering; only relevant if
 *	 the 'DPNI_OPT_DIST_FS' option was set at DPNI creation
 */
enum dpni_dist_mode {
	DPNI_DIST_MODE_NONE = 0,
	DPNI_DIST_MODE_HASH = 1,
	DPNI_DIST_MODE_FS = 2
};

/**
 * enum dpni_fs_miss_action -   DPNI Flow Steering miss action
 * @DPNI_FS_MISS_DROP: In case of no-match, drop the frame
 * @DPNI_FS_MISS_EXPLICIT_FLOWID: In case of no-match, use explicit flow-id
 * @DPNI_FS_MISS_HASH: In case of no-match, distribute using hash
 */
enum dpni_fs_miss_action {
	DPNI_FS_MISS_DROP = 0,
	DPNI_FS_MISS_EXPLICIT_FLOWID = 1,
	DPNI_FS_MISS_HASH = 2
};

/**
 * struct dpni_fs_tbl_cfg - Flow Steering table configuration
 * @miss_action: Miss action selection
 * @default_flow_id: Used when 'miss_action = DPNI_FS_MISS_EXPLICIT_FLOWID'
 */
struct dpni_fs_tbl_cfg {
	enum dpni_fs_miss_action	miss_action;
	u16				default_flow_id;
};

int dpni_prepare_key_cfg(const struct dpkg_profile_cfg *cfg,
			 u8 *key_cfg_buf);

/**
 * enum dpni_tx_schedule_mode - DPNI Tx scheduling mode
 * @DPNI_TX_SCHED_STRICT_PRIORITY: strict priority
 * @DPNI_TX_SCHED_WEIGHTED_A: weighted based scheduling in group A
 * @DPNI_TX_SCHED_WEIGHTED_B: weighted based scheduling in group B
 */
enum dpni_tx_schedule_mode {
	DPNI_TX_SCHED_STRICT_PRIORITY = 0,
	DPNI_TX_SCHED_WEIGHTED_A,
	DPNI_TX_SCHED_WEIGHTED_B,
};

/**
 * struct dpni_tx_schedule_cfg - Structure representing Tx scheduling conf
 * @mode:		Scheduling mode
 * @delta_bandwidth:	Bandwidth represented in weights from 100 to 10000;
 *	not applicable for 'strict-priority' mode;
 */
struct dpni_tx_schedule_cfg {
	enum dpni_tx_schedule_mode mode;
	u16 delta_bandwidth;
};

/**
 * struct dpni_tx_priorities_cfg - Structure representing transmission
 *					priorities for DPNI TCs
 * @tc_sched: An array of traffic-classes which should be used in the
 * following way:
 *   - If max_tx_tcs <= 8: the tc_sched[n] struct will host the configuration
 *   requested for TC#n
 *   - If max_tx_tcs > 8: the tc_sched[n] struct will host the configuration
 *   requeted for TC#(8 + n). In this case, the first 8 TCs are configured by
 *   MC in strict priority order and cannot be changed.
 *   The only accepted configuration in this case is:
 *    - TCs [8-12) will be part of WEIGHTED_A group
 *    - TCs [12-16) will be part of WEIGHTED_B group
 *   Any other configuration will get rejected by the MC firmware. The
 *   delta_bandwidth for each TC can be used as usual.
 * @prio_group_A: Priority of group A
 * @prio_group_B: Priority of group B
 * @separate_groups: Treat A and B groups as separate
 * @ceetm_ch_idx: ceetm channel index to apply the changes
 */
struct dpni_tx_priorities_cfg {
	struct dpni_tx_schedule_cfg tc_sched[DPNI_MAX_TC];
	u8 prio_group_A;
	u8 prio_group_B;
	u8 separate_groups;
};

int dpni_set_tx_priorities(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   const struct dpni_tx_priorities_cfg *cfg);

/**
 * struct dpni_rx_tc_dist_cfg - Rx traffic class distribution configuration
 * @dist_size: Set the distribution size;
 *	supported values: 1,2,3,4,6,7,8,12,14,16,24,28,32,48,56,64,96,
 *	112,128,192,224,256,384,448,512,768,896,1024
 * @dist_mode: Distribution mode
 * @key_cfg_iova: I/O virtual address of 256 bytes DMA-able memory filled with
 *		the extractions to be used for the distribution key by calling
 *		dpni_prepare_key_cfg() relevant only when
 *		'dist_mode != DPNI_DIST_MODE_NONE', otherwise it can be '0'
 * @fs_cfg: Flow Steering table configuration; only relevant if
 *		'dist_mode = DPNI_DIST_MODE_FS'
 */
struct dpni_rx_tc_dist_cfg {
	u16			dist_size;
	enum dpni_dist_mode	dist_mode;
	u64			key_cfg_iova;
	struct dpni_fs_tbl_cfg	fs_cfg;
};

int dpni_set_rx_tc_dist(struct fsl_mc_io			*mc_io,
			u32					cmd_flags,
			u16					token,
			u8					tc_id,
			const struct dpni_rx_tc_dist_cfg	*cfg);

/**
 * DPNI_FS_MISS_DROP - When used for fs_miss_flow_id in function
 * dpni_set_rx_dist, will signal to dpni to drop all unclassified frames
 */
#define DPNI_FS_MISS_DROP		((uint16_t)-1)

/**
 * struct dpni_rx_dist_cfg - Rx distribution configuration
 * @dist_size:	distribution size
 * @key_cfg_iova: I/O virtual address of 256 bytes DMA-able memory filled with
 *		the extractions to be used for the distribution key by calling
 *		dpni_prepare_key_cfg(); relevant only when enable!=0 otherwise
 *		it can be '0'
 * @enable: enable/disable the distribution.
 * @tc: TC id for which distribution is set
 * @fs_miss_flow_id: when packet misses all rules from flow steering table and
 *		hash is disabled it will be put into this queue id; use
 *		DPNI_FS_MISS_DROP to drop frames. The value of this field is
 *		used only when flow steering distribution is enabled and hash
 *		distribution is disabled
 */
struct dpni_rx_dist_cfg {
	u16 dist_size;
	u64 key_cfg_iova;
	u8 enable;
	u8 tc;
	u16 fs_miss_flow_id;
};

int dpni_set_rx_fs_dist(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			const struct dpni_rx_dist_cfg *cfg);

int dpni_set_rx_hash_dist(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  const struct dpni_rx_dist_cfg *cfg);

/**
 * struct dpni_qos_tbl_cfg - Structure representing QOS table configuration
 * @key_cfg_iova: I/O virtual address of 256 bytes DMA-able memory filled with
 *		key extractions to be used as the QoS criteria by calling
 *		dpkg_prepare_key_cfg()
 * @discard_on_miss: Set to '1' to discard frames in case of no match (miss);
 *		'0' to use the 'default_tc' in such cases
 * @default_tc: Used in case of no-match and 'discard_on_miss'= 0
 */
struct dpni_qos_tbl_cfg {
	u64 key_cfg_iova;
	int discard_on_miss;
	u8 default_tc;
};

int dpni_set_qos_table(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       const struct dpni_qos_tbl_cfg *cfg);

/**
 * enum dpni_dest - DPNI destination types
 * @DPNI_DEST_NONE: Unassigned destination; The queue is set in parked mode and
 *		does not generate FQDAN notifications; user is expected to
 *		dequeue from the queue based on polling or other user-defined
 *		method
 * @DPNI_DEST_DPIO: The queue is set in schedule mode and generates FQDAN
 *		notifications to the specified DPIO; user is expected to dequeue
 *		from the queue only after notification is received
 * @DPNI_DEST_DPCON: The queue is set in schedule mode and does not generate
 *		FQDAN notifications, but is connected to the specified DPCON
 *		object; user is expected to dequeue from the DPCON channel
 */
enum dpni_dest {
	DPNI_DEST_NONE = 0,
	DPNI_DEST_DPIO = 1,
	DPNI_DEST_DPCON = 2
};

/**
 * struct dpni_queue - Queue structure
 * @destination: - Destination structure
 * @destination.id: ID of the destination, only relevant if DEST_TYPE is > 0.
 *	Identifies either a DPIO or a DPCON object.
 *	Not relevant for Tx queues.
 * @destination.type:	May be one of the following:
 *	0 - No destination, queue can be manually
 *		queried, but will not push traffic or
 *		notifications to a DPIO;
 *	1 - The destination is a DPIO. When traffic
 *		becomes available in the queue a FQDAN
 *		(FQ data available notification) will be
 *		generated to selected DPIO;
 *	2 - The destination is a DPCON. The queue is
 *		associated with a DPCON object for the
 *		purpose of scheduling between multiple
 *		queues. The DPCON may be independently
 *		configured to generate notifications.
 *		Not relevant for Tx queues.
 * @destination.hold_active: Hold active, maintains a queue scheduled for longer
 *	in a DPIO during dequeue to reduce spread of traffic.
 *	Only relevant if queues are
 *	not affined to a single DPIO.
 * @user_context: User data, presented to the user along with any frames
 *	from this queue. Not relevant for Tx queues.
 * @flc: FD FLow Context structure
 * @flc.value: Default FLC value for traffic dequeued from
 *      this queue.  Please check description of FD
 *      structure for more information.
 *      Note that FLC values set using dpni_add_fs_entry,
 *      if any, take precedence over values per queue.
 * @flc.stash_control: Boolean, indicates whether the 6 lowest
 *      - significant bits are used for stash control.
 *      significant bits are used for stash control.  If set, the 6
 *      least significant bits in value are interpreted as follows:
 *      - bits 0-1: indicates the number of 64 byte units of context
 *      that are stashed.  FLC value is interpreted as a memory address
 *      in this case, excluding the 6 LS bits.
 *      - bits 2-3: indicates the number of 64 byte units of frame
 *      annotation to be stashed.  Annotation is placed at FD[ADDR].
 *      - bits 4-5: indicates the number of 64 byte units of frame
 *      data to be stashed.  Frame data is placed at FD[ADDR] +
 *      FD[OFFSET].
 *      For more details check the Frame Descriptor section in the
 *      hardware documentation.
 */
struct dpni_queue {
	struct {
		u16 id;
		enum dpni_dest type;
		char hold_active;
		u8 priority;
	} destination;
	u64 user_context;
	struct {
		u64 value;
		char stash_control;
	} flc;
};

/**
 * struct dpni_queue_id - Queue identification, used for enqueue commands
 *			or queue control
 * @fqid: FQID used for enqueueing to and/or configuration of this specific FQ
 * @qdbin: Queueing bin, used to enqueue using QDID, DQBIN, QPRI. Only relevant
 *		for Tx queues.
 */
struct dpni_queue_id {
	u32 fqid;
	u16 qdbin;
};

/* Set User Context */
#define DPNI_QUEUE_OPT_USER_CTX		0x00000001
#define DPNI_QUEUE_OPT_DEST		0x00000002
#define DPNI_QUEUE_OPT_FLC		0x00000004
#define DPNI_QUEUE_OPT_HOLD_ACTIVE	0x00000008

int dpni_set_queue(struct fsl_mc_io	*mc_io,
		   u32			cmd_flags,
		   u16			token,
		   enum dpni_queue_type	qtype,
		   u8			tc,
		   u8			index,
		   u8			options,
		   const struct dpni_queue *queue);

int dpni_get_queue(struct fsl_mc_io	*mc_io,
		   u32			cmd_flags,
		   u16			token,
		   enum dpni_queue_type	qtype,
		   u8			tc,
		   u8			index,
		   struct dpni_queue	*queue,
		   struct dpni_queue_id	*qid);

/**
 * enum dpni_congestion_unit - DPNI congestion units
 * @DPNI_CONGESTION_UNIT_BYTES: bytes units
 * @DPNI_CONGESTION_UNIT_FRAMES: frames units
 */
enum dpni_congestion_unit {
	DPNI_CONGESTION_UNIT_BYTES = 0,
	DPNI_CONGESTION_UNIT_FRAMES
};

/**
 * enum dpni_congestion_point - Structure representing congestion point
 * @DPNI_CP_QUEUE: Set taildrop per queue, identified by QUEUE_TYPE, TC and
 *		QUEUE_INDEX
 * @DPNI_CP_GROUP: Set taildrop per queue group. Depending on options used to
 *		define the DPNI this can be either per TC (default) or per
 *		interface (DPNI_OPT_SHARED_CONGESTION set at DPNI create).
 *		QUEUE_INDEX is ignored if this type is used.
 */
enum dpni_congestion_point {
	DPNI_CP_QUEUE,
	DPNI_CP_GROUP,
};

/**
 * struct dpni_dest_cfg - Structure representing DPNI destination parameters
 * @dest_type:	Destination type
 * @dest_id:	Either DPIO ID or DPCON ID, depending on the destination type
 * @priority:	Priority selection within the DPIO or DPCON channel; valid
 *		values are 0-1 or 0-7, depending on the number of priorities
 *		in that channel; not relevant for 'DPNI_DEST_NONE' option
 */
struct dpni_dest_cfg {
	enum dpni_dest dest_type;
	int dest_id;
	u8 priority;
};

/* DPNI congestion options */

/**
 * DPNI_CONG_OPT_FLOW_CONTROL - This congestion will trigger flow control or
 * priority flow control.  This will have effect only if flow control is
 * enabled with dpni_set_link_cfg().
 */
#define DPNI_CONG_OPT_FLOW_CONTROL		0x00000040

/**
 * struct dpni_congestion_notification_cfg - congestion notification
 *					configuration
 * @units: Units type
 * @threshold_entry: Above this threshold we enter a congestion state.
 *		set it to '0' to disable it
 * @threshold_exit: Below this threshold we exit the congestion state.
 * @message_ctx: The context that will be part of the CSCN message
 * @message_iova: I/O virtual address (must be in DMA-able memory),
 *		must be 16B aligned; valid only if 'DPNI_CONG_OPT_WRITE_MEM_<X>'
 *		is contained in 'options'
 * @dest_cfg: CSCN can be send to either DPIO or DPCON WQ channel
 * @notification_mode: Mask of available options; use 'DPNI_CONG_OPT_<X>' values
 */

struct dpni_congestion_notification_cfg {
	enum dpni_congestion_unit units;
	u32 threshold_entry;
	u32 threshold_exit;
	u64 message_ctx;
	u64 message_iova;
	struct dpni_dest_cfg dest_cfg;
	u16 notification_mode;
};

/** Compose TC parameter for function dpni_set_congestion_notification()
 * and dpni_get_congestion_notification().
 */
#define DPNI_BUILD_CH_TC(ceetm_ch_idx, tc) \
	((((ceetm_ch_idx) & 0x0F) << 4) | ((tc) & 0x0F))

int dpni_set_congestion_notification(
			struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			enum dpni_queue_type qtype,
			u8 tc_id,
			const struct dpni_congestion_notification_cfg *cfg);

/**
 * struct dpni_taildrop - Structure representing the taildrop
 * @enable:	Indicates whether the taildrop is active or not.
 * @units:	Indicates the unit of THRESHOLD. Queue taildrop only supports
 *		byte units, this field is ignored and assumed = 0 if
 *		CONGESTION_POINT is 0.
 * @threshold:	Threshold value, in units identified by UNITS field. Value 0
 *		cannot be used as a valid taildrop threshold, THRESHOLD must
 *		be > 0 if the taildrop is enabled.
 */
struct dpni_taildrop {
	char enable;
	enum dpni_congestion_unit units;
	u32 threshold;
};

int dpni_set_taildrop(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      enum dpni_congestion_point cg_point,
		      enum dpni_queue_type q_type,
		      u8 tc,
		      u8 q_index,
		      struct dpni_taildrop *taildrop);

int dpni_get_taildrop(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      enum dpni_congestion_point cg_point,
		      enum dpni_queue_type q_type,
		      u8 tc,
		      u8 q_index,
		      struct dpni_taildrop *taildrop);

/**
 * struct dpni_rule_cfg - Rule configuration for table lookup
 * @key_iova: I/O virtual address of the key (must be in DMA-able memory)
 * @mask_iova: I/O virtual address of the mask (must be in DMA-able memory)
 * @key_size: key and mask size (in bytes)
 */
struct dpni_rule_cfg {
	u64	key_iova;
	u64	mask_iova;
	u8	key_size;
};

/**
 * DPNI_FS_OPT_DISCARD - Discard matching traffic. If set, this takes
 * precedence over any other configuration and matching traffic is always
 * discarded.
 */
 #define DPNI_FS_OPT_DISCARD            0x1

/**
 * DPNI_FS_OPT_SET_FLC - Set FLC value. If set, flc member of struct
 * dpni_fs_action_cfg is used to override the FLC value set per queue.
 * For more details check the Frame Descriptor section in the hardware
 * documentation.
 */
#define DPNI_FS_OPT_SET_FLC            0x2

/**
 * DPNI_FS_OPT_SET_STASH_CONTROL - Indicates whether the 6 lowest significant
 * bits of FLC are used for stash control. If set, the 6 least significant bits
 * in value are interpreted as follows:
 *     - bits 0-1: indicates the number of 64 byte units of context that are
 *     stashed. FLC value is interpreted as a memory address in this case,
 *     excluding the 6 LS bits.
 *     - bits 2-3: indicates the number of 64 byte units of frame annotation
 *     to be stashed. Annotation is placed at FD[ADDR].
 *     - bits 4-5: indicates the number of 64 byte units of frame data to be
 *     stashed. Frame data is placed at FD[ADDR] + FD[OFFSET].
 * This flag is ignored if DPNI_FS_OPT_SET_FLC is not specified.
 */
#define DPNI_FS_OPT_SET_STASH_CONTROL  0x4

/**
 * struct dpni_fs_action_cfg - Action configuration for table look-up
 * @flc:	FLC value for traffic matching this rule. Please check the
 *		Frame Descriptor section in the hardware documentation for
 *		more information.
 * @flow_id:	Identifies the Rx queue used for matching traffic. Supported
 *		values are in range 0 to num_queue-1.
 * @options:	Any combination of DPNI_FS_OPT_ values.
 */
struct dpni_fs_action_cfg {
	u64 flc;
	u16 flow_id;
	u16 options;
};

int dpni_add_fs_entry(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      u8 tc_id,
		      u16 index,
		      const struct dpni_rule_cfg *cfg,
		      const struct dpni_fs_action_cfg *action);

int dpni_remove_fs_entry(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u8 tc_id,
			 const struct dpni_rule_cfg *cfg);

int dpni_add_qos_entry(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       const struct dpni_rule_cfg *cfg,
		       u8 tc_id,
		       u16 index);

int dpni_remove_qos_entry(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  const struct dpni_rule_cfg *cfg);

int dpni_clear_qos_table(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token);

int dpni_get_api_version(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 *major_ver,
			 u16 *minor_ver);
/**
 * struct dpni_tx_shaping_cfg - Structure representing DPNI tx shaping configuration
 * @rate_limit:		Rate in Mbps
 * @max_burst_size:	Burst size in bytes (up to 64KB)
 */
struct dpni_tx_shaping_cfg {
	u32 rate_limit;
	u16 max_burst_size;
};

int dpni_set_tx_shaping(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			const struct dpni_tx_shaping_cfg *tx_cr_shaper,
			const struct dpni_tx_shaping_cfg *tx_er_shaper,
			int coupled);

/**
 * struct dpni_single_step_cfg - configure single step PTP (IEEE 1588)
 * @en:		enable single step PTP. When enabled the PTPv1 functionality
 *		will not work. If the field is zero, offset and ch_update
 *		parameters will be ignored
 * @offset:	start offset from the beginning of the frame where
 *		timestamp field is found. The offset must respect all MAC
 *		headers, VLAN tags and other protocol headers
 * @ch_update:	when set UDP checksum will be updated inside packet
 * @peer_delay:	For peer-to-peer transparent clocks add this value to the
 *		correction field in addition to the transient time update.
 *		The value expresses nanoseconds.
 * @ptp_onestep_reg_base: 1588 SINGLE_STEP register base address. This address
 *			  is used to update directly the register contents.
 *			  User has to create an address mapping for it.
 *
 *
 */
struct dpni_single_step_cfg {
	u8	en;
	u8	ch_update;
	u16	offset;
	u32	peer_delay;
	u32	ptp_onestep_reg_base;
};

int dpni_set_single_step_cfg(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     struct dpni_single_step_cfg *ptp_cfg);

int dpni_get_single_step_cfg(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     struct dpni_single_step_cfg *ptp_cfg);

int dpni_enable_vlan_filter(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			    u32 en);

int dpni_add_vlan_id(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
		     u16 vlan_id, u8 flags, u8 tc_id, u8 flow_id);

int dpni_remove_vlan_id(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			u16 vlan_id);

int dpni_is_macsec_capable(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			   int *macsec_capable);

/**
 * enum macsec_validation_mode - validation function for received frames
 *
 * @MACSEC_SECY_VALIDATION_DISABLE: disable the validation function
 * @MACSEC_SECY_VALIDATION_CHECK: enable the validation function but only for
 *      checking without filtering out invalid frames
 * @MACSEC_SECY_VALIDATION_STRICT: enable the validation function and also
 *      strictly filter out those invalid frames
 */
enum macsec_validation_mode {
	MACSEC_SECY_VALIDATION_DISABLE = 0x00,
	MACSEC_SECY_VALIDATION_CHECK = 0x01,
	MACSEC_SECY_VALIDATION_STRICT = 0x02
};

/* enum macsec_cipher_suite - Cipher Suite used for protecting transmitted
 *      frames and decrypting received frames
 *
 * @MACSEC_CIPHER_SUITE_GCM_AES_128: GCM-AES-128
 * @MACSEC_CIPHER_SUITE_GCM_AES_256: GCM-AES-256
 */
enum macsec_cipher_suite {
	MACSEC_CIPHER_SUITE_GCM_AES_128 = 0x00,
	MACSEC_CIPHER_SUITE_GCM_AES_256 = 0x01
};

/**
 *struct macsec_cipher_suite_cfg - Cipher Suite configuration
 *
 *@cipher_suite                         Cipher Suite
 *@confidentiality:                     '1' for enabling confidentiality
 *                                      protection; 0 for disabling
 *@confidentiality_offset:              Number of bytes from the frame data
 *                                      start that are not confidentiality
 *                                      protected
 */
struct macsec_cipher_suite_cfg {
	enum macsec_cipher_suite cipher_suite;
	int confidentiality;
	u8 co_offset;
};

/**
 * struct macsec_secy_cfg - SecY configuration
 *
 * @cs: Cipher Suite configuration
 * @tx_sci: SCI of transmitting channel. It should be composed of 48-bytes
 *          source MAC address concatenated with 16-bit port id.  In case of
 *          point-to-point mode, this field has no meaning.
 * @is_ptp: Point-to-Point mode. In this mode the SCI is not presented in the
 *          outgoing frame; Instead, the second end-point should be configured
 *          to point-to-point mode as well and be set with the same key.
 * @validation_mode: Validation mode for received frames.
 * @max_rx_sc: Maximum number of receiving-SC that can be created on this SecY.
 *             Ignored in point-to-point mode.
 */
struct macsec_secy_cfg {
	struct macsec_cipher_suite_cfg cs;
	u64 tx_sci;
	int is_ptp;
	enum macsec_validation_mode validation_mode;
	u8 max_rx_sc;
};

int dpni_add_secy(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
		  const struct macsec_secy_cfg *cfg, u8 *secy_id);

int dpni_remove_secy(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token, u8 secy_id);

int dpni_secy_set_state(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			u8 secy_id, bool active);

int dpni_secy_set_tx_protection(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
				u8 secy_id, bool protect);

int dpni_secy_set_replay_protection(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
				    u8 secy_id, bool en, u32 window);

/**
 * struct macsec_tx_sa_cfg - transmitting-SA configuration
 *
 * @an: Association Number (AN). 2-bits value (0-3)
 * @key: Key used for protecting outgoing frames
 * @next_pn: The PN field value for the first outgoing frame from this SA
 */
struct macsec_tx_sa_cfg {
	u8 key[32];
	u32 next_pn;
	u8 an;
};

int dpni_secy_add_tx_sa(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			u8 secy_id, struct macsec_tx_sa_cfg *cfg);

int dpni_secy_remove_tx_sa(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			   u8 secy_id, u8 an);

int dpni_secy_set_active_tx_sa(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			       u8 secy_id, u8 assoc_num);

int dpni_secy_add_rx_sc(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			u8 secy_id, u64 sci);

int dpni_secy_remove_rx_sc(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			   u8 secy_id, u64 sci);

int dpni_secy_set_rx_sc_state(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			      u8 secy_id, u64 sci, bool active);

/**
 * struct macsec_rx_sa_cfg - receiving-SA configuration
 *
 * @key: Key used for decrypting received frames
 * @lowest_pn: Initial lowest PN field allowed for received frames
 * @an: Association Number (AN). 2-bits value (0-3)
 */
struct macsec_rx_sa_cfg {
	u8 key[32];
	u32 lowest_pn;
	u8 an;
};

int dpni_secy_add_rx_sa(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token, u8 secy_id,
			u64 sci, struct macsec_rx_sa_cfg *cfg);

int dpni_secy_remove_rx_sa(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			   u8 secy_id, u64 sci, u8 an);

int dpni_secy_set_rx_sa_next_pn(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
				u8 secy_id, u64 sci, u8 an, u32 next_pn);

int dpni_secy_set_rx_sa_state(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			      u8 secy_id, u64 sci, u8 an, bool active);

/**
 * union macsec_secy_stats - per SecY statistics
 *
 * @page_0: Page_0 statistics structure
 * @page_0.cnt_ing_bytes: Count ingress bytes on the controlled port
 * @page_0.cnt_ing_ucast_frames: Count ingress unicast-frames on the controlled port
 * @page_0.cnt_ing_mcast_frames: Count ingress multicast-frames on the controlled port
 * @page_0.cnt_ing_bcast_frames: Count ingress broadcast-frames on the controlled port
 * @page_0.cnt_egr_bytes: Count egress bytes
 * @page_0.cnt_egr_ucast_frames: Count egress unicast-frames
 * @page_0.cnt_egr_mcast_frames: Count egress multicast-frames
 *
 * @page_1: Page_1 statistics structure
 * @page_1.cnt_egr_bcast_frames: Count egress broadcast-frames
 *
 */
union macsec_secy_stats {
	struct {
		u64 cnt_ing_bytes;
		u64 cnt_ing_ucast_frames;
		u64 cnt_ing_mcast_frames;
		u64 cnt_ing_bcast_frames;
		u64 cnt_egr_bytes;
		u64 cnt_egr_ucast_frames;
		u64 cnt_egr_mcast_frames;
	} page_0;
	struct {
		u64 cnt_egr_bcast_frames;
	} page_1;
	struct {
		u64 counter[DPNI_STATISTICS_CNT];
	} raw;
};

int dpni_secy_get_stats(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			u8 secy_id, u8 page, union macsec_secy_stats *stats);

/**
 * union macsec_secy_tx_sc_stats - per Tx SC statistics
 *
 * @page_0: Page_0 statistics structure
 * @page_0.protected_frames: Count protected frames.
 * @page_0.encrypted_frames: Count encrypted frames.
 * @page_0.protected_bytes: Count total bytes of all protected frames.
 * @page_0.encrypted_bytes: Count total bytes of all encrypted frames.
 */
union macsec_secy_tx_sc_stats {
	struct {
		u64 protected_frames;
		u64 encrypted_frames;
		u64 protected_bytes;
		u64 encrypted_bytes;
	} page_0;
	struct {
		u64 counter[DPNI_STATISTICS_CNT];
	} raw;
};

int dpni_secy_get_tx_sc_stats(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			      u8 secy_id, union macsec_secy_tx_sc_stats *stats);

/**
 * union macsec_secy_tx_sa_stats - per Tx SA statistics
 *
 * @page_0: Page_0 statistics structure
 * @page_0.protected_frames: Count protected frames.
 * @page_0.encrypted_frames: Count encrypted frames.
 */
union macsec_secy_tx_sa_stats {
	struct {
		u32 protected_frames;
		u32 encrypted_frames;
	} page_0;
	struct {
		u32 counter[DPNI_STATISTICS_32_CNT];
	} raw;
};

int dpni_secy_get_tx_sa_stats(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			      u8 secy_id, u8 an, union macsec_secy_tx_sa_stats *stats);

/**
 * union macsec_secy_rx_sc_stats - per Rx SC statistics
 *
 * @page_0: Page_0 statistics structure
 * @page_0.unused_frames: Count frames received on the SC's disabled SAs where
 *                        the SecY validateFrame property is not 'Strict' and
 *                        the frame C bit is not set
 * @page_0.not_using_sa_frames: Count frames received on the SC's disabled SAs
 *                              where the SecY validateFrame property is
 *                              'Strict' or the frame C bit is set
 * @page_0.invalid_frames: Count frames that failed integrity check where the
 *                         SecY validateFrame property is 'Check'
 * @page_0.not_valid_frames: Count frames that failed integrity check where the
 *                           SecY validateFrame property is 'Strict' or the
 *                           frame C bit is set
 * @page_0.late_frames: Count frames discarded on the SC's SA due to PN field
 *                      value lower then the SA lowestPN property where the
 *                      SecY replay protection is enabled
 * @page_0.delayed_frames: Count frames discarded on the SC's SA due to PN
 *                         field value lower then the SA lowestPN property
 *                         where the SecY replay protection is disabled and
 *                         where the frame does not satisfying the conditions
 *                         for increasing MACSEC_SECY_CNT_RX_SC_INVALID_FRAME
 *                         or MACSEC_SECY_CNT_RX_SC_NOT_VALID_FRAME counters
 * @page_0.unchecked_frames: Count frames that failed integrity check having C
 *                           bit not set where the SecY validateFrame property
 *                           is 'Disabled'
 *
 * @page_1: Page_1 statistics structure
 * @page_1.ok_frames: Count frames that passed integrity check having PN field
 *                    value above the SA lowestPN property
 * @page_1.validated_bytes: Count bytes of frames that passed integrity check
 *                          having E bit not set where the SecY validateFrame
 *                          property is not 'Disabled'
 * @page_1.decrypted_bytes: Count bytes of frames that passed integrity check
 *                          having E bit set where the SecY validateFrame
 *                          property is not 'Disabled'
 */
union macsec_secy_rx_sc_stats {
	struct {
		u64 unused_frames;
		u64 not_using_sa_frames;
		u64 invalid_frames;
		u64 not_valid_frames;
		u64 late_frames;
		u64 delayed_frames;
		u64 unchecked_frames;
	} page_0;
	struct {
		u64 ok_frames;
		u64 validated_bytes;
		u64 decrypted_bytes;
	} page_1;
	struct {
		u64 counter[DPNI_STATISTICS_CNT];
	} raw;
};

int dpni_secy_get_rx_sc_stats(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			      u8 secy_id, u64 sci, u8 page,
			      union macsec_secy_rx_sc_stats *stats);

/**
 * enum macsec_secy_rx_sa_counter - per Rx SA statistics
 *
 * @page_0.unused_sa_frames: Count frames received when the SA is disabled
 *                           where the SecY validateFrame property is not
 *                           'Strict' and the frame C bit is not set
 * @page_0.not_using_sa_frames: Count frames received when the SA is disabled
 *                              where the SecY validateFrame property is
 *                              'Strict' or the frame C bit is set
 * @page_0.invalid_frames: Count frames that failed integrity check where the
 *                         SecY validateFrame property is 'Check'
 * @page_0.not_valid_frames: Count frames that failed integrity check where the
 *                           SecY validateFrame property is 'Strict' or the
 *                           frame C bit is set
 * @page_0.ok_frames: Count frames that passed integrity check having PN field
 *                    value above the SA lowestPN property
 */
union macsec_secy_rx_sa_stats {
	struct {
		u32 unused_sa_frames;
		u32 not_using_sa_frames;
		u32 invalid_frames;
		u32 not_valid_frames;
		u32 ok_frames;
	} page_0;
	struct {
		u32 counter[DPNI_STATISTICS_32_CNT];
	} raw;
};

int dpni_secy_get_rx_sa_stats(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			      u8 secy_id, u64 sci, u8 an, union macsec_secy_rx_sa_stats *stats);

union macsec_global_stats {
	struct {
		u32 in_without_tag_frames;
		u32 in_kay_frames;
		u32 in_bag_tag_frames;
		u32 in_sci_not_found_frames;
		u32 in_unsupported_ec_frames;
		u32 in_too_long_frames;
		u32 out_discarded_frames;
	} page_0;
	struct {
		u32 counter[DPNI_STATISTICS_32_CNT];
	} raw;
};

int dpni_get_macsec_stats(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			  union macsec_global_stats *stats);
#endif /* __FSL_DPNI_H */
