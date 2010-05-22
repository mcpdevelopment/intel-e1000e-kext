/*******************************************************************************
 
 Intel PRO/1000 Linux driver
 Copyright(c) 1999 - 2010 Intel Corporation.
 
 This program is free software; you can redistribute it and/or modify it
 under the terms and conditions of the GNU General Public License,
 version 2, as published by the Free Software Foundation.
 
 This program is distributed in the hope it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 more details.
 
 You should have received a copy of the GNU General Public License along with
 this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 
 The full GNU General Public License is included in this distribution in
 the file called "COPYING".
 
 Contact Information:
 Linux NICS <linux.nics@intel.com>
 e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
 Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 
 *******************************************************************************/

#ifndef _E1000_HW_H_
#define _E1000_HW_H_

#include <libkern/OSByteOrder.h>
#include <libkern/OSAtomic.h>
#include <IOKit/IOLib.h>

#include "e1000_regs.h"
#include "e1000_defines.h"


#ifdef DEBUG
#define e_debug(fmt, arg...) IOLog("IntelE1000e debug: " fmt "\n", ## arg)
#define e_dbg(fmt, arg...) IOLog("IntelE1000e debug: " fmt, ## arg)
#else
#define e_debug(format, arg...) do {  } while (0)
#define e_dbg(format, arg...) do {  } while (0)
#endif
#define e_info(fmt, arg...) IOLog("IntelE1000e info: " fmt, ## arg)
#define usec_delay(x) IODelay(x)                                             
#define msec_delay(x) IODelay(x*1000)
#define msleep(x) msec_delay(x)
#define mdelay(x) msec_delay(x)
#define udelay(x) usec_delay(x)



struct e1000_hw;

#define E1000_DEV_ID_82571EB_COPPER           0x105E
#define E1000_DEV_ID_82571EB_FIBER            0x105F
#define E1000_DEV_ID_82571EB_SERDES           0x1060
#define E1000_DEV_ID_82571EB_SERDES_DUAL      0x10D9
#define E1000_DEV_ID_82571EB_SERDES_QUAD      0x10DA
#define E1000_DEV_ID_82571EB_QUAD_COPPER      0x10A4
#define E1000_DEV_ID_82571PT_QUAD_COPPER      0x10D5
#define E1000_DEV_ID_82571EB_QUAD_FIBER       0x10A5
#define E1000_DEV_ID_82571EB_QUAD_COPPER_LP   0x10BC
#define E1000_DEV_ID_82572EI_COPPER           0x107D
#define E1000_DEV_ID_82572EI_FIBER            0x107E
#define E1000_DEV_ID_82572EI_SERDES           0x107F
#define E1000_DEV_ID_82572EI                  0x10B9
#define E1000_DEV_ID_82573E                   0x108B
#define E1000_DEV_ID_82573E_IAMT              0x108C
#define E1000_DEV_ID_82573L                   0x109A
#define E1000_DEV_ID_82574L                   0x10D3
#define E1000_DEV_ID_82574LA                  0x10F6
#define E1000_DEV_ID_82583V                   0x150C
#define E1000_DEV_ID_80003ES2LAN_COPPER_DPT   0x1096
#define E1000_DEV_ID_80003ES2LAN_SERDES_DPT   0x1098
#define E1000_DEV_ID_80003ES2LAN_COPPER_SPT   0x10BA
#define E1000_DEV_ID_80003ES2LAN_SERDES_SPT   0x10BB
#define E1000_DEV_ID_ICH8_82567V_3            0x1501
#define E1000_DEV_ID_ICH8_IGP_M_AMT           0x1049
#define E1000_DEV_ID_ICH8_IGP_AMT             0x104A
#define E1000_DEV_ID_ICH8_IGP_C               0x104B
#define E1000_DEV_ID_ICH8_IFE                 0x104C
#define E1000_DEV_ID_ICH8_IFE_GT              0x10C4
#define E1000_DEV_ID_ICH8_IFE_G               0x10C5
#define E1000_DEV_ID_ICH8_IGP_M               0x104D
#define E1000_DEV_ID_ICH9_IGP_M               0x10BF
#define E1000_DEV_ID_ICH9_IGP_M_AMT           0x10F5
#define E1000_DEV_ID_ICH9_IGP_M_V             0x10CB
#define E1000_DEV_ID_ICH9_IGP_AMT             0x10BD
#define E1000_DEV_ID_ICH9_BM                  0x10E5
#define E1000_DEV_ID_ICH9_IGP_C               0x294C
#define E1000_DEV_ID_ICH9_IFE                 0x10C0
#define E1000_DEV_ID_ICH9_IFE_GT              0x10C3
#define E1000_DEV_ID_ICH9_IFE_G               0x10C2
#define E1000_DEV_ID_ICH10_R_BM_LM            0x10CC
#define E1000_DEV_ID_ICH10_R_BM_LF            0x10CD
#define E1000_DEV_ID_ICH10_R_BM_V             0x10CE
#define E1000_DEV_ID_ICH10_D_BM_LM            0x10DE
#define E1000_DEV_ID_ICH10_D_BM_LF            0x10DF
#define E1000_DEV_ID_PCH_M_HV_LM              0x10EA
#define E1000_DEV_ID_PCH_M_HV_LC              0x10EB
#define E1000_DEV_ID_PCH_D_HV_DM              0x10EF
#define E1000_DEV_ID_PCH_D_HV_DC              0x10F0
#define E1000_REVISION_0 0
#define E1000_REVISION_1 1
#define E1000_REVISION_2 2
#define E1000_REVISION_3 3
#define E1000_REVISION_4 4

#define E1000_FUNC_0     0
#define E1000_FUNC_1     1

#define E1000_ALT_MAC_ADDRESS_OFFSET_LAN0   0
#define E1000_ALT_MAC_ADDRESS_OFFSET_LAN1   3

typedef enum {
	e1000_undefined = 0,
	e1000_82571,
	e1000_82572,
	e1000_82573,
	e1000_82574,
	e1000_82583,
	e1000_80003es2lan,
	e1000_ich8lan,
	e1000_ich9lan,
	e1000_ich10lan,
	e1000_pchlan,
	e1000_num_macs  /* List is 1-based, so subtract 1 for true count. */
} e1000_mac_type;

enum e1000_media_type {
	e1000_media_type_unknown = 0,
	e1000_media_type_copper = 1,
	e1000_media_type_fiber = 2,
	e1000_media_type_internal_serdes = 3,
	e1000_num_media_types
};

enum e1000_nvm_type {
	e1000_nvm_unknown = 0,
	e1000_nvm_none,
	e1000_nvm_eeprom_spi,
	e1000_nvm_flash_hw,
	e1000_nvm_flash_sw
};

enum e1000_nvm_override {
	e1000_nvm_override_none = 0,
	e1000_nvm_override_spi_small,
	e1000_nvm_override_spi_large,
};

enum e1000_phy_type {
	e1000_phy_unknown = 0,
	e1000_phy_none,
	e1000_phy_m88,
	e1000_phy_igp,
	e1000_phy_igp_2,
	e1000_phy_gg82563,
	e1000_phy_igp_3,
	e1000_phy_ife,
	e1000_phy_bm,
	e1000_phy_82578,
	e1000_phy_82577,
};

enum e1000_bus_type {
	e1000_bus_type_unknown = 0,
	e1000_bus_type_pci,
	e1000_bus_type_pcix,
	e1000_bus_type_pci_express,
	e1000_bus_type_reserved
};

enum e1000_bus_speed {
	e1000_bus_speed_unknown = 0,
	e1000_bus_speed_33,
	e1000_bus_speed_66,
	e1000_bus_speed_100,
	e1000_bus_speed_120,
	e1000_bus_speed_133,
	e1000_bus_speed_2500,
	e1000_bus_speed_5000,
	e1000_bus_speed_reserved
};

enum e1000_bus_width {
	e1000_bus_width_unknown = 0,
	e1000_bus_width_pcie_x1,
	e1000_bus_width_pcie_x2,
	e1000_bus_width_pcie_x4 = 4,
	e1000_bus_width_pcie_x8 = 8,
	e1000_bus_width_32,
	e1000_bus_width_64,
	e1000_bus_width_reserved
};

enum e1000_1000t_rx_status {
	e1000_1000t_rx_status_not_ok = 0,
	e1000_1000t_rx_status_ok,
	e1000_1000t_rx_status_undefined = 0xFF
};

enum e1000_rev_polarity {
	e1000_rev_polarity_normal = 0,
	e1000_rev_polarity_reversed,
	e1000_rev_polarity_undefined = 0xFF
};

enum e1000_fc_mode {
	e1000_fc_none = 0,
	e1000_fc_rx_pause,
	e1000_fc_tx_pause,
	e1000_fc_full,
	e1000_fc_default = 0xFF
};

enum e1000_ms_type {
	e1000_ms_hw_default = 0,
	e1000_ms_force_master,
	e1000_ms_force_slave,
	e1000_ms_auto
};

enum e1000_smart_speed {
	e1000_smart_speed_default = 0,
	e1000_smart_speed_on,
	e1000_smart_speed_off
};

enum e1000_serdes_link_state {
	e1000_serdes_link_down = 0,
	e1000_serdes_link_autoneg_progress,
	e1000_serdes_link_autoneg_complete,
	e1000_serdes_link_forced_up
};

/* Receive Descriptor */
struct e1000_rx_desc {
	UInt64 buffer_addr; /* Address of the descriptor's data buffer */
	UInt16 length;      /* Length of data DMAed into data buffer */
	UInt16 csum;        /* Packet checksum */
	UInt8  status;         /* Descriptor status */
	UInt8  errors;         /* Descriptor Errors */
	UInt16 special;
};

/* Receive Descriptor - Extended */
union e1000_rx_desc_extended {
	struct {
		UInt64 buffer_addr;
		UInt64 reserved;
	} read;
	struct {
		struct {
			UInt32 mrq;           /* Multiple Rx Queues */
			union {
				UInt32 rss;         /* RSS Hash */
				struct {
					UInt16 ip_id;  /* IP id */
					UInt16 csum;   /* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			UInt32 status_error;  /* ext status/error */
			UInt16 length;
			UInt16 vlan;          /* VLAN tag */
		} upper;
	} wb;  /* writeback */
};

#define MAX_PS_BUFFERS 4
/* Receive Descriptor - Packet Split */
union e1000_rx_desc_packet_split {
	struct {
		/* one buffer for protocol header(s), three data buffers */
		UInt64 buffer_addr[MAX_PS_BUFFERS];
	} read;
	struct {
		struct {
			UInt32 mrq;           /* Multiple Rx Queues */
			union {
				UInt32 rss;           /* RSS Hash */
				struct {
					UInt16 ip_id;    /* IP id */
					UInt16 csum;     /* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			UInt32 status_error;  /* ext status/error */
			UInt16 length0;       /* length of buffer 0 */
			UInt16 vlan;          /* VLAN tag */
		} middle;
		struct {
			UInt16 header_status;
			UInt16 length[3];     /* length of buffers 1-3 */
		} upper;
		UInt64 reserved;
	} wb; /* writeback */
};

/* Transmit Descriptor */
struct e1000_tx_desc {
	UInt64 buffer_addr;   /* Address of the descriptor's data buffer */
	union {
		UInt32 data;
		struct {
			UInt16 length;    /* Data buffer length */
			UInt8 cso;           /* Checksum offset */
			UInt8 cmd;           /* Descriptor control */
		} flags;
	} lower;
	union {
		UInt32 data;
		struct {
			UInt8 status;        /* Descriptor status */
			UInt8 css;           /* Checksum start */
			UInt16 special;
		} fields;
	} upper;
};

/* Offload Context Descriptor */
struct e1000_context_desc {
	union {
		UInt32 ip_config;
		struct {
			UInt8 ipcss;         /* IP checksum start */
			UInt8 ipcso;         /* IP checksum offset */
			UInt16 ipcse;     /* IP checksum end */
		} ip_fields;
	} lower_setup;
	union {
		UInt32 tcp_config;
		struct {
			UInt8 tucss;         /* TCP checksum start */
			UInt8 tucso;         /* TCP checksum offset */
			UInt16 tucse;     /* TCP checksum end */
		} tcp_fields;
	} upper_setup;
	UInt32 cmd_and_length;
	union {
		UInt32 data;
		struct {
			UInt8 status;        /* Descriptor status */
			UInt8 hdr_len;       /* Header length */
			UInt16 mss;       /* Maximum segment size */
		} fields;
	} tcp_seg_setup;
};

/* Offload data descriptor */
struct e1000_data_desc {
	UInt64 buffer_addr;   /* Address of the descriptor's buffer address */
	union {
		UInt32 data;
		struct {
			UInt16	length;    /* Data buffer length */
			UInt8 typ_len_ext;
			UInt8 cmd;
		} flags;
	} lower;
	union {
		UInt32 data;
		struct {
			UInt8 status;        /* Descriptor status */
			UInt8 popts;         /* Packet Options */
			UInt16 special;
		} fields;
	} upper;
};

/* Statistics counters collected by the MAC */
struct e1000_hw_stats {
	UInt64 crcerrs;
	UInt64 algnerrc;
	UInt64 symerrs;
	UInt64 rxerrc;
	UInt64 mpc;
	UInt64 scc;
	UInt64 ecol;
	UInt64 mcc;
	UInt64 latecol;
	UInt64 colc;
	UInt64 dc;
	UInt64 tncrs;
	UInt64 sec;
	UInt64 cexterr;
	UInt64 rlec;
	UInt64 xonrxc;
	UInt64 xontxc;
	UInt64 xoffrxc;
	UInt64 xofftxc;
	UInt64 fcruc;
	UInt64 prc64;
	UInt64 prc127;
	UInt64 prc255;
	UInt64 prc511;
	UInt64 prc1023;
	UInt64 prc1522;
	UInt64 gprc;
	UInt64 bprc;
	UInt64 mprc;
	UInt64 gptc;
	UInt64 gorc;
	UInt64 gotc;
	UInt64 rnbc;
	UInt64 ruc;
	UInt64 rfc;
	UInt64 roc;
	UInt64 rjc;
	UInt64 mgprc;
	UInt64 mgpdc;
	UInt64 mgptc;
	UInt64 tor;
	UInt64 tot;
	UInt64 tpr;
	UInt64 tpt;
	UInt64 ptc64;
	UInt64 ptc127;
	UInt64 ptc255;
	UInt64 ptc511;
	UInt64 ptc1023;
	UInt64 ptc1522;
	UInt64 mptc;
	UInt64 bptc;
	UInt64 tsctc;
	UInt64 tsctfc;
	UInt64 iac;
	UInt64 icrxptc;
	UInt64 icrxatc;
	UInt64 ictxptc;
	UInt64 ictxatc;
	UInt64 ictxqec;
	UInt64 ictxqmtc;
	UInt64 icrxdmtc;
	UInt64 icrxoc;
	UInt64 doosync;
};


struct e1000_phy_stats {
	UInt32 idle_errors;
	UInt32 receive_errors;
};

struct e1000_host_mng_dhcp_cookie {
	UInt32 signature;
	UInt8  status;
	UInt8  reserved0;
	UInt16 vlan_id;
	UInt32 reserved1;
	UInt16 reserved2;
	UInt8  reserved3;
	UInt8  checksum;
};

/* Host Interface "Rev 1" */
struct e1000_host_command_header {
	UInt8 command_id;
	UInt8 command_length;
	UInt8 command_options;
	UInt8 checksum;
};

#define E1000_HI_MAX_DATA_LENGTH     252
struct e1000_host_command_info {
	struct e1000_host_command_header command_header;
	UInt8 command_data[E1000_HI_MAX_DATA_LENGTH];
};

/* Host Interface "Rev 2" */
struct e1000_host_mng_command_header {
	UInt8  command_id;
	UInt8  checksum;
	UInt16 reserved1;
	UInt16 reserved2;
	UInt16 command_length;
};

#define E1000_HI_MAX_MNG_DATA_LENGTH 0x6F8
struct e1000_host_mng_command_info {
	struct e1000_host_mng_command_header command_header;
	UInt8 command_data[E1000_HI_MAX_MNG_DATA_LENGTH];
};

#include "e1000_mac.h"
#include "e1000_phy.h"
#include "e1000_nvm.h"
#include "e1000_manage.h"

struct e1000_mac_operations {
	/* Function pointers for the MAC. */
	SInt32  (*init_params)(struct e1000_hw *);
	SInt32  (*id_led_init)(struct e1000_hw *);
	SInt32  (*blink_led)(struct e1000_hw *);
	SInt32  (*check_for_link)(struct e1000_hw *);
	bool (*check_mng_mode)(struct e1000_hw *hw);
	SInt32  (*cleanup_led)(struct e1000_hw *);
	void (*clear_hw_cntrs)(struct e1000_hw *);
	void (*clear_vfta)(struct e1000_hw *);
	SInt32  (*get_bus_info)(struct e1000_hw *);
	void (*set_lan_id)(struct e1000_hw *);
	SInt32  (*get_link_up_info)(struct e1000_hw *, UInt16 *, UInt16 *);
	SInt32  (*led_on)(struct e1000_hw *);
	SInt32  (*led_off)(struct e1000_hw *);
	void (*update_mc_addr_list)(struct e1000_hw *, UInt8 *, UInt32);
	SInt32  (*reset_hw)(struct e1000_hw *);
	SInt32  (*init_hw)(struct e1000_hw *);
	SInt32  (*setup_link)(struct e1000_hw *);
	SInt32  (*setup_physical_interface)(struct e1000_hw *);
	SInt32  (*setup_led)(struct e1000_hw *);
	void (*write_vfta)(struct e1000_hw *, UInt32, UInt32);
	void (*mta_set)(struct e1000_hw *, UInt32);
	void (*config_collision_dist)(struct e1000_hw *);
	void (*rar_set)(struct e1000_hw *, UInt8*, UInt32);
	SInt32  (*read_mac_addr)(struct e1000_hw *);
	SInt32  (*validate_mdi_setting)(struct e1000_hw *);
	SInt32  (*mng_host_if_write)(struct e1000_hw *, UInt8*, UInt16, UInt16, UInt8*);
	SInt32  (*mng_write_cmd_header)(struct e1000_hw *hw,
								 struct e1000_host_mng_command_header*);
	SInt32  (*mng_enable_host_if)(struct e1000_hw *);
	SInt32  (*wait_autoneg)(struct e1000_hw *);
};

struct e1000_phy_operations {
	SInt32  (*init_params)(struct e1000_hw *);
	SInt32  (*acquire)(struct e1000_hw *);
	SInt32  (*cfg_on_link_up)(struct e1000_hw *);
	SInt32  (*check_polarity)(struct e1000_hw *);
	SInt32  (*check_reset_block)(struct e1000_hw *);
	SInt32  (*commit)(struct e1000_hw *);
	SInt32  (*force_speed_duplex)(struct e1000_hw *);
	SInt32  (*get_cfg_done)(struct e1000_hw *hw);
	SInt32  (*get_cable_length)(struct e1000_hw *);
	SInt32  (*get_info)(struct e1000_hw *);
	SInt32  (*read_reg)(struct e1000_hw *, UInt32, UInt16 *);
	SInt32  (*read_reg_locked)(struct e1000_hw *, UInt32, UInt16 *);
	void (*release)(struct e1000_hw *);
	SInt32  (*reset)(struct e1000_hw *);
	SInt32  (*set_d0_lplu_state)(struct e1000_hw *, bool);
	SInt32  (*set_d3_lplu_state)(struct e1000_hw *, bool);
	SInt32  (*write_reg)(struct e1000_hw *, UInt32, UInt16);
	SInt32  (*write_reg_locked)(struct e1000_hw *, UInt32, UInt16);
	void (*power_up)(struct e1000_hw *);
	void (*power_down)(struct e1000_hw *);
};

struct e1000_nvm_operations {
	SInt32  (*init_params)(struct e1000_hw *);
	SInt32  (*acquire)(struct e1000_hw *);
	SInt32  (*read)(struct e1000_hw *, UInt16, UInt16, UInt16 *);
	void (*release)(struct e1000_hw *);
	void (*reload)(struct e1000_hw *);
	SInt32  (*update)(struct e1000_hw *);
	SInt32  (*valid_led_default)(struct e1000_hw *, UInt16 *);
	SInt32  (*validate)(struct e1000_hw *);
	SInt32  (*write)(struct e1000_hw *, UInt16, UInt16, UInt16 *);
};

struct e1000_mac_info {
	struct e1000_mac_operations ops;
	UInt8 addr[6];
	UInt8 perm_addr[6];
	
	e1000_mac_type type;
	
	UInt32 collision_delta;
	UInt32 ledctl_default;
	UInt32 ledctl_mode1;
	UInt32 ledctl_mode2;
	UInt32 mc_filter_type;
	UInt32 tx_packet_delta;
	UInt32 txcw;
	
	UInt16 current_ifs_val;
	UInt16 ifs_max_val;
	UInt16 ifs_min_val;
	UInt16 ifs_ratio;
	UInt16 ifs_step_size;
	UInt16 mta_reg_count;
	
	/* Maximum size of the MTA register table in all supported adapters */
#define MAX_MTA_REG 128
	UInt32 mta_shadow[MAX_MTA_REG];
	UInt16 rar_entry_count;
	
	UInt8  forced_speed_duplex;
	
	bool adaptive_ifs;
	bool arc_subsystem_valid;
	bool asf_firmware_present;
	bool autoneg;
	bool autoneg_failed;
	bool get_link_status;
	bool in_ifs_mode;
	enum e1000_serdes_link_state serdes_link_state;
	bool serdes_has_link;
	bool tx_pkt_filtering;
};

struct e1000_phy_info {
	struct e1000_phy_operations ops;
	enum e1000_phy_type type;
	
	enum e1000_1000t_rx_status local_rx;
	enum e1000_1000t_rx_status remote_rx;
	enum e1000_ms_type ms_type;
	enum e1000_ms_type original_ms_type;
	enum e1000_rev_polarity cable_polarity;
	enum e1000_smart_speed smart_speed;
	
	UInt32 addr;
	UInt32 id;
	UInt32 reset_delay_us; /* in usec */
	UInt32 revision;
	
	enum e1000_media_type media_type;
	
	UInt16 autoneg_advertised;
	UInt16 autoneg_mask;
	UInt16 cable_length;
	UInt16 max_cable_length;
	UInt16 min_cable_length;
	
	UInt8 mdix;
	
	bool disable_polarity_correction;
	bool is_mdix;
	bool polarity_correction;
	bool reset_disable;
	bool speed_downgraded;
	bool autoneg_wait_to_complete;
};

struct e1000_nvm_info {
	struct e1000_nvm_operations ops;
	enum e1000_nvm_type type;
	enum e1000_nvm_override override;
	
	UInt32 flash_bank_size;
	UInt32 flash_base_addr;
	
	UInt16 word_size;
	UInt16 delay_usec;
	UInt16 address_bits;
	UInt16 opcode_bits;
	UInt16 page_size;
};

struct e1000_bus_info {
	enum e1000_bus_type type;
	enum e1000_bus_speed speed;
	enum e1000_bus_width width;
	
	UInt16 func;
	UInt16 pci_cmd_word;
};

struct e1000_fc_info {
	UInt32 high_water;          /* Flow control high-water mark */
	UInt32 low_water;           /* Flow control low-water mark */
	UInt16 pause_time;          /* Flow control pause timer */
	bool send_xon;           /* Flow control send XON */
	bool strict_ieee;        /* Strict IEEE mode */
	enum e1000_fc_mode current_mode; /* FC mode in effect */
	enum e1000_fc_mode requested_mode; /* FC mode requested by caller */
	enum e1000_fc_mode type; /* Type of flow control */
	enum e1000_fc_mode original_type;
};

struct e1000_dev_spec_82571 {
	bool laa_is_present;
	UInt32 smb_counter;
};

struct e1000_dev_spec_80003es2lan {
	bool  mdic_wa_enable;
};

struct e1000_shadow_ram {
	UInt16  value;
	bool modified;
};

#define E1000_ICH8_SHADOW_RAM_WORDS		2048

struct e1000_dev_spec_ich8lan {
	bool kmrn_lock_loss_workaround_enabled;
	struct e1000_shadow_ram shadow_ram[E1000_ICH8_SHADOW_RAM_WORDS];
	bool nvm_k1_enabled;
};

struct e1000_hw {
	struct e1000_adapter *adapter;
	
//	UInt8 __iomem *hw_addr;
//	UInt8 __iomem *flash_address;
	void *hw_addr;
	void *flash_address;
	
	struct e1000_mac_info  mac;
	struct e1000_fc_info   fc;
	struct e1000_phy_info  phy;
	struct e1000_nvm_info  nvm;
	struct e1000_bus_info  bus;
	struct e1000_host_mng_dhcp_cookie mng_cookie;
	
	union {
		struct e1000_dev_spec_82571	_82571;
		struct e1000_dev_spec_80003es2lan _80003es2lan;
		struct e1000_dev_spec_ich8lan	ich8lan;
	} dev_spec;
	UInt32 dev_spec_size; //++
	
	UInt16 device_id;
	UInt16 subsystem_vendor_id;
	UInt16 subsystem_device_id;
	UInt16 vendor_id;
	
	UInt8  revision_id;
	
	UInt16 pci_header_type;  //++
	UInt16 pcie_link_status; //++
};

#include "e1000_82571.h"
#include "e1000_80003es2lan.h"
#include "e1000_ich8lan.h"

/* These functions must be implemented by drivers */
SInt32  e1000_read_pcie_cap_reg(struct e1000_hw *hw, UInt32 reg, UInt16 *value);

#endif
