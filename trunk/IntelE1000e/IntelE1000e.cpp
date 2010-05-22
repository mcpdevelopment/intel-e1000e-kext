/*
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
 */
/*
 *
 * based on Intel82566MM.cpp by Guijin Ding
 *
 * IntelE1000e.cpp
 *
 */

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/network/IOMbufMemoryCursor.h>
#include <IOKit/network/IOPacketQueue.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/assert.h>

extern "C" {
#include <sys/kpi_mbuf.h>
#include <net/ethernet.h>
}

extern "C" {
#include "e1000.h"
}

#include "IntelE1000e.h"

#define TBDS_PER_TCB 12
#define super IOEthernetController

static struct e1000_info e1000_ich8_info = {
	e1000_ich8lan,
	FLAG_HAS_WOL
	| FLAG_IS_ICH
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_AMT
	| FLAG_HAS_FLASH
	| FLAG_APME_IN_WUC,
	8,
	ETH_FRAME_LEN + ETH_FCS_LEN,
	e1000_get_variants_ich8lan,
	e1000_init_function_pointers_ich8lan
};


static struct e1000_info e1000_ich9_info = {
	e1000_ich9lan,
	FLAG_HAS_JUMBO_FRAMES
	| FLAG_IS_ICH
	| FLAG_HAS_WOL
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_AMT
	| FLAG_HAS_ERT
	| FLAG_HAS_FLASH
	| FLAG_APME_IN_WUC,
	10,
	DEFAULT_JUMBO,
	e1000_get_variants_ich8lan,
	e1000_init_function_pointers_ich8lan
};

static struct e1000_info e1000_ich10_info = {
	e1000_ich10lan,
	FLAG_HAS_JUMBO_FRAMES
	| FLAG_IS_ICH
	| FLAG_HAS_WOL
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_AMT
	| FLAG_HAS_ERT
	| FLAG_HAS_FLASH
	| FLAG_APME_IN_WUC,
	10,
	DEFAULT_JUMBO,
	e1000_get_variants_ich8lan,
	e1000_init_function_pointers_ich8lan,
};

static struct e1000_info e1000_pch_info = {
	e1000_pchlan,
	FLAG_IS_ICH
	| FLAG_HAS_WOL
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_AMT
	| FLAG_HAS_FLASH
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_DISABLE_FC_PAUSE_TIME /* errata */
	| FLAG_APME_IN_WUC,
	26,
	4096,
	e1000_get_variants_ich8lan,
	e1000_init_function_pointers_ich8lan,
};

static struct e1000_info e1000_es2_info = {
	e1000_80003es2lan,
	FLAG_HAS_HW_VLAN_FILTER
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_RX_NEEDS_RESTART /* errata */
	| FLAG_TARC_SET_BIT_ZERO /* errata */
	| FLAG_APME_CHECK_PORT_B
	| FLAG_DISABLE_FC_PAUSE_TIME /* errata */
	| FLAG_TIPG_MEDIUM_FOR_80003ESLAN,
	38,
	DEFAULT_JUMBO,
	NULL,
	e1000_init_function_pointers_80003es2lan,
};

static struct e1000_info e1000_82571_info = {
	e1000_82571,
	FLAG_HAS_HW_VLAN_FILTER
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_HAS_SMART_POWER_DOWN
	| FLAG_RESET_OVERWRITES_LAA /* errata */
	| FLAG_TARC_SPEED_MODE_BIT /* errata */
	| FLAG_APME_CHECK_PORT_B,
	38,
	DEFAULT_JUMBO,
	NULL,
	e1000_init_function_pointers_82571,
};

static struct e1000_info e1000_82572_info = {
	e1000_82572,
	FLAG_HAS_HW_VLAN_FILTER
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_CTRLEXT_ON_LOAD
	| FLAG_TARC_SPEED_MODE_BIT, /* errata */
	38,
	DEFAULT_JUMBO,
	NULL,
	e1000_init_function_pointers_82571,
};

static struct e1000_info e1000_82573_info = {
	e1000_82573,
	FLAG_HAS_HW_VLAN_FILTER
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_SMART_POWER_DOWN
	| FLAG_HAS_AMT
	| FLAG_HAS_ERT
	| FLAG_HAS_SWSM_ON_LOAD,
	20,
	ETH_FRAME_LEN + ETH_FCS_LEN,
	NULL,
	e1000_init_function_pointers_82571,
};      

static struct e1000_info e1000_82574_info = {
	e1000_82574,
	FLAG_HAS_HW_VLAN_FILTER
#ifdef CONFIG_E1000E_MSIX
	| FLAG_HAS_MSIX
#endif
	| FLAG_HAS_JUMBO_FRAMES
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_SMART_POWER_DOWN
	| FLAG_HAS_AMT
	| FLAG_HAS_CTRLEXT_ON_LOAD,
	20,
	DEFAULT_JUMBO,
	NULL,
	e1000_init_function_pointers_82571,
};      

static struct e1000_info e1000_82583_info = {
	e1000_82583,
	FLAG_HAS_HW_VLAN_FILTER
	| FLAG_HAS_WOL
	| FLAG_APME_IN_CTRL3
	| FLAG_RX_CSUM_ENABLED
	| FLAG_HAS_SMART_POWER_DOWN
	| FLAG_HAS_AMT
	| FLAG_HAS_CTRLEXT_ON_LOAD,
	20,
	ETH_FRAME_LEN + ETH_FCS_LEN,
	NULL,
	e1000_init_function_pointers_82571,
};      

static inline void RELEASE(OSObject* x)
{
	if(x != NULL) {
		x->release();
	} 
}

static inline struct e1000_info* GET_EI(unsigned short device_id)
{
	struct adapter_id_info {
		unsigned short int device_id;
		struct e1000_info* info;
	} e1000e_pci_tbl[] = {
		{ E1000_DEV_ID_82571EB_COPPER, &e1000_82571_info },
		{ E1000_DEV_ID_82571EB_FIBER, &e1000_82571_info },
		{ E1000_DEV_ID_82571EB_QUAD_COPPER, &e1000_82571_info },
		{ E1000_DEV_ID_82571EB_QUAD_COPPER_LP, &e1000_82571_info },
		{ E1000_DEV_ID_82571EB_QUAD_FIBER, &e1000_82571_info },
		{ E1000_DEV_ID_82571EB_SERDES, &e1000_82571_info },
		{ E1000_DEV_ID_82571EB_SERDES_DUAL, &e1000_82571_info },
		{ E1000_DEV_ID_82571EB_SERDES_QUAD, &e1000_82571_info },
		{ E1000_DEV_ID_82571PT_QUAD_COPPER, &e1000_82571_info },
		
		{ E1000_DEV_ID_82572EI, &e1000_82572_info },
		{ E1000_DEV_ID_82572EI_COPPER, &e1000_82572_info },
		{ E1000_DEV_ID_82572EI_FIBER, &e1000_82572_info },
		{ E1000_DEV_ID_82572EI_SERDES, &e1000_82572_info },
		
		{ E1000_DEV_ID_82573E, &e1000_82573_info },
		{ E1000_DEV_ID_82573E_IAMT, &e1000_82573_info },
		{ E1000_DEV_ID_82573L, &e1000_82573_info },
		
		{ E1000_DEV_ID_82574L, &e1000_82574_info },
		{ E1000_DEV_ID_82574LA, &e1000_82574_info },
		{ E1000_DEV_ID_82583V, &e1000_82583_info },
		
		{ E1000_DEV_ID_80003ES2LAN_COPPER_DPT, &e1000_es2_info },
		{ E1000_DEV_ID_80003ES2LAN_COPPER_SPT, &e1000_es2_info },
		{ E1000_DEV_ID_80003ES2LAN_SERDES_DPT, &e1000_es2_info },
		{ E1000_DEV_ID_80003ES2LAN_SERDES_SPT, &e1000_es2_info },
		
		{ E1000_DEV_ID_ICH8_IFE, &e1000_ich8_info },
		{ E1000_DEV_ID_ICH8_IFE_G, &e1000_ich8_info },
		{ E1000_DEV_ID_ICH8_IFE_GT, &e1000_ich8_info },
		{ E1000_DEV_ID_ICH8_IGP_AMT, &e1000_ich8_info },
		{ E1000_DEV_ID_ICH8_IGP_C, &e1000_ich8_info },
		{ E1000_DEV_ID_ICH8_IGP_M, &e1000_ich8_info },
		{ E1000_DEV_ID_ICH8_IGP_M_AMT, &e1000_ich8_info },
		{ E1000_DEV_ID_ICH8_82567V_3, &e1000_ich8_info },
		
		{ E1000_DEV_ID_ICH9_IFE, &e1000_ich9_info },
		{ E1000_DEV_ID_ICH9_IFE_G, &e1000_ich9_info },
		{ E1000_DEV_ID_ICH9_IFE_GT, &e1000_ich9_info },
		{ E1000_DEV_ID_ICH9_IGP_AMT, &e1000_ich9_info },
		{ E1000_DEV_ID_ICH9_IGP_C, &e1000_ich9_info },
		{ E1000_DEV_ID_ICH9_BM, &e1000_ich9_info },
		{ E1000_DEV_ID_ICH9_IGP_M, &e1000_ich9_info },
		{ E1000_DEV_ID_ICH9_IGP_M_AMT, &e1000_ich9_info },
		{ E1000_DEV_ID_ICH9_IGP_M_V, &e1000_ich9_info },
		
		{ E1000_DEV_ID_ICH10_R_BM_LM, &e1000_ich9_info },
		{ E1000_DEV_ID_ICH10_R_BM_LF, &e1000_ich9_info },
		{ E1000_DEV_ID_ICH10_R_BM_V, &e1000_ich9_info },
		
		{ E1000_DEV_ID_ICH10_D_BM_LM, &e1000_ich10_info },
		{ E1000_DEV_ID_ICH10_D_BM_LF, &e1000_ich10_info },
		
		{ E1000_DEV_ID_PCH_M_HV_LM, &e1000_pch_info },
		{ E1000_DEV_ID_PCH_M_HV_LC, &e1000_pch_info },
		{ E1000_DEV_ID_PCH_D_HV_DM, &e1000_pch_info },
		{ E1000_DEV_ID_PCH_D_HV_DC, &e1000_pch_info },
		
		{0, NULL }	/* terminate list */
	};
	
	struct e1000_info* ei = &e1000_ich8_info;
	int i = 0;
	while (1) {
		if (e1000e_pci_tbl[i].info == NULL)
			break;
		if (e1000e_pci_tbl[i].device_id == device_id) {
			ei = e1000e_pci_tbl[i].info;
			break;
		}
		i++;
	}
	return ei;			
}

OSDefineMetaClassAndStructors(IntelE1000e, super);

void IntelE1000e::free()
{
	e_debug("void IntelE1000e::free()");
	
	RELEASE(netif);
	RELEASE(interruptSource);
	RELEASE(watchdogSource);
	
	RELEASE(workLoop);
	RELEASE(pciDevice);
	RELEASE(mediumDict);
	
	int i;
	for (i = 0; i < MEDIUM_INDEX_COUNT; i++) {
		RELEASE(mediumTable[i]);
	}
	
	RELEASE(csrPCIAddress);
	RELEASE(flashPCIAddress);
	
	RELEASE(rxMbufCursor);
	RELEASE(txMbufCursor);
	
//	if (adapter.hw.dev_spec) {
//		IOFree(adapter.hw.dev_spec, adapter.hw.dev_spec_size);
//		adapter.hw.dev_spec = NULL;
//	}
	
	super::free();
	return;
}

bool IntelE1000e::init(OSDictionary *properties)
{
	e_debug("bool IntelE1000e::Init(OSDictionary *properties).");
	
	if (super::init(properties) == false) 
		return false;
	int i;
	
	enabledForNetif = false;
	interruptEnabled = false;
	
	pciDevice = NULL;
	mediumDict = NULL;
	csrPCIAddress = NULL;
	flashPCIAddress = NULL;
	interruptSource = NULL;
	watchdogSource = NULL;
	netif = NULL;
	
	transmitQueue = NULL;
	preLinkStatus = 0;
	rxMbufCursor = NULL;
	txMbufCursor = NULL;
	
	for (i = 0; i < MEDIUM_INDEX_COUNT; i++) {
		mediumTable[i] = NULL;
	}
	
	return true;
}
void IntelE1000e::stop(IOService* provider)
{
	e_debug("IntelE1000e::stop(IOService * provider)");
	
	if (watchdogSource && workLoop) {
		workLoop->removeEventSource(watchdogSource);
	}
	
	if (interruptSource && workLoop) {
		workLoop->removeEventSource(interruptSource);
	}
	
	super::stop(provider);
	return;
}

bool IntelE1000e::start(IOService * provider)
{
	e_debug("IntelE1000e::start(IOService * provider)");
	
	UInt16 reg16;
	struct e1000_info* ei = NULL;
	if (super::start(provider) == false) {
		e_debug("supper::start failed.");
		return false;
	}
	
  	transmitQueue = getOutputQueue();
	if (transmitQueue == NULL) {
		e_debug("getOutputQueue failed.");
		goto FAILED_STARTED;
	}
	
	pciDevice = OSDynamicCast(IOPCIDevice, provider);
	if (pciDevice == NULL) {
		e_debug("cast");
		goto FAILED_STARTED;
	}
	
	pciDevice->retain();
	if (pciDevice->open(this) == false) {
		e_debug("open");
		goto FAILED_STARTED;
	}
	
	if (pciDevice->requestPowerDomainState(
										   kIOPMPowerOn,
										   (IOPowerConnection *) getParentEntry(gIOPowerPlane),
										   IOPMLowestState ) != IOPMNoErr )
	{
		e_debug("power.");
		goto FAILED_OPENED;
	}
	
	
	reg16 = pciDevice->configRead16( kIOPCIConfigCommand );
	
	reg16 |= ( kIOPCICommandBusMaster       |
			  kIOPCICommandMemorySpace     |
			  kIOPCICommandMemWrInvalidate );
	reg16 &= ~kIOPCICommandIOSpace;  // disable I/O space
	pciDevice->configWrite16( kIOPCIConfigCommand, reg16 );
	
	reg16 = pciDevice->configRead16(kIOPCIConfigVendorID);
	e_info("vendorid = 0x%x   ", reg16);
	reg16 = pciDevice->configRead16(kIOPCIConfigDeviceID);
	e_info("deviceid = 0x%x \n", reg16);
	adapter.hw.device_id = reg16;
	reg16 = pciDevice->configRead16(PCI_HEADER_TYPE_REGISTER);
	adapter.hw.pci_header_type = reg16;
	
	if (e1000_read_pcie_cap_reg(&adapter.hw,
								PCIE_LINK_STATUS,
								&adapter.hw.pcie_link_status)) {
		adapter.hw.pcie_link_status = 0;
	}
	
	csrPCIAddress = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
	if (csrPCIAddress == NULL) {
		e_debug("csrPCIAddress.");
		goto FAILED_OPENED;
	}
	flashPCIAddress = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress1);
	if (flashPCIAddress == NULL) {
		e_debug("flashPCIAddress.");
		goto FAILED_OPENED;
	}
	
	csrCPUAddress = csrPCIAddress->getVirtualAddress();
	flashCPUAddress = flashPCIAddress->getVirtualAddress();
	
	ei = GET_EI(adapter.hw.device_id);
	if (ei == NULL) {
		e_debug("GET_EI.");
		goto FAILED_OPENED;
	}
	
	adapter.ei = ei;
//	adapter.hw.back = &adapter;
	adapter.hw.mac.type = ei->mac;
	adapter.hw.hw_addr = (void*)csrCPUAddress;
	adapter.hw.flash_address = (void*)flashCPUAddress;
	
	//ETHER_MAX_LEN -> 1518, 2 -> VLAN_HEADER_LEN, 4 -> CRC_LEN
	adapter.rx_buffer_len = ETHER_MAX_LEN + 2 + 4;
	adapter.max_frame_size = ETHER_MAX_LEN;
	adapter.min_frame_size = ETHER_MIN_LEN;
	
	adapter.ei->init_ops(&adapter.hw);
	adapter.hw.mac.ops.init_params(&adapter.hw);
	adapter.hw.nvm.ops.init_params(&adapter.hw);
	adapter.hw.phy.ops.init_params(&adapter.hw);
	
	//Explicitly disable IRQ since the NIC can be in any state.
	E1000_WRITE_REG(&adapter.hw, E1000_IMC, ~0);
	E1000_READ_REG(&adapter.hw, E1000_STATUS);
	//sw_init end
	
	adapter.flags |= FLAG_LSC_GIG_SPEED_DROP;
	e1000e_get_bus_info_pcie(&adapter.hw);
	if (adapter.ei->mac == e1000_ich8lan) {
		if (adapter.hw.bus.width == e1000_bus_width_unknown)
			adapter.hw.bus.width = e1000_bus_width_pcie_x1;
	}
	
	
	adapter.hw.phy.autoneg_wait_to_complete = 0;	
	/* Copper options */
	if (adapter.hw.phy.media_type == e1000_media_type_copper) {
		adapter.hw.phy.mdix = AUTO_ALL_MODES;
		adapter.hw.phy.disable_polarity_correction = 0;
		adapter.hw.phy.ms_type = e1000_ms_hw_default;
	}
	
	if (adapter.hw.phy.ops.check_reset_block &&
		adapter.hw.phy.ops.check_reset_block(&adapter.hw))
		e_info("PHY reset is blocked due to SOL/IDER session.\n");
	
	adapter.hw.mac.ops.reset_hw(&adapter.hw);
	
	int i;
	for (i = 0;; i++) {
		if (adapter.hw.nvm.ops.validate(&adapter.hw) >= 0)
			break;
		
		if (i == 2) {
			e_debug("The NVM Checksum Is Not Valid");
			goto FAILED_OPENED;
		}
	}
	
	/* copy the MAC address out of the NVM */
	if (e1000e_read_mac_addr(&adapter.hw))
		e_debug("NVM Read Error while reading MAC address");
	
	/*
	 e_info("mac address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	 adapter.hw.mac.addr[0], adapter.hw.mac.addr[1], 
	 adapter.hw.mac.addr[2], adapter.hw.mac.addr[3], 
	 adapter.hw.mac.addr[4], adapter.hw.mac.addr[5]);
	 */
	
	// Initialize link parameters. User can change them with ethtool
	adapter.hw.mac.autoneg = 1;
	adapter.fc_autoneg = 1;
	adapter.hw.fc.original_type = e1000_fc_default;
	adapter.hw.fc.type = e1000_fc_default;
	adapter.hw.phy.autoneg_advertised = 0x2f;
	
	// reset the hardware with the new settings
	e1000_reset();
	
	// Get a handle to our superclass' workloop.
	//
	IOWorkLoop * myWorkLoop = (IOWorkLoop *) getWorkLoop();
	if (!myWorkLoop) {
		e_debug(" myWorkLoop is NULL.");
		goto FAILED_OPENED;
	}
	
	mediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT + 1);
	if (mediumDict == NULL) {
		e_debug("mediumDict .");
		goto FAILED_OPENED;
	}
	
	if (!addNetworkMedium(kIOMediumEthernetAuto, 0, MEDIUM_INDEX_AUTO)) {
		e_debug("eeAddNetworkMedium : %d.", MEDIUM_INDEX_AUTO);
		goto FAILED_OPENED;
	}
	if (!addNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex,
						  10 * MBit, MEDIUM_INDEX_10HD)) {
		e_debug("eeAddNetworkMedium : %d.", MEDIUM_INDEX_10HD);
		goto FAILED_OPENED;
	}
	if (!addNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex,
						  10 * MBit, MEDIUM_INDEX_10FD)) {
		e_debug("eeAddNetworkMedium : %d.", MEDIUM_INDEX_10FD);
		goto FAILED_OPENED;
	}
	if (!addNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex,
						  100 * MBit, MEDIUM_INDEX_100HD)) {
		e_debug("eeAddNetworkMedium : %d.", MEDIUM_INDEX_100HD);
		goto FAILED_OPENED;
	}
	if (!addNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex,
						  100 * MBit, MEDIUM_INDEX_100FD)) {
		e_debug("eeAddNetworkMedium : %d.", MEDIUM_INDEX_100FD);
		goto FAILED_OPENED;
	}
	if (!addNetworkMedium(kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex,
						  1000 * MBit, MEDIUM_INDEX_1000FD)) {
		e_debug("eeAddNetworkMedium : %d.", MEDIUM_INDEX_1000FD);
		goto FAILED_OPENED;
	}
	
	if (!publishMediumDictionary(mediumDict)) {
		e_debug("publishMediumDictionary.");
		goto FAILED_OPENED;
	}
	
	//Attaching dynamic link layer
	if (attachInterface((IONetworkInterface **)(&netif), false) == false) {
		e_debug("Failed to attach data link layer.");
		goto FAILED_OPENED;
	}
	netif->registerService();
	
	
	interruptSource = IOFilterInterruptEventSource::filterInterruptEventSource(
																			   this,
																			   &IntelE1000e::interruptHandler,
																			   &IntelE1000e::interruptFilter,
																			   provider);
	
	if (!interruptSource ||
	    (myWorkLoop->addEventSource(interruptSource) != kIOReturnSuccess)) {
		e_debug("workloop add eventsource interrupt source.");
		goto FAILED_OPENED;
	}
	
	// This is important. If the interrupt line is shared with other devices,
	// then the interrupt vector will be enabled only if all corresponding
	// interrupt event sources are enabled. To avoid masking interrupts for
	// other devices that are sharing the interrupt line, the event source
	// is enabled immediately.
	interruptSource->enable();
	
	// Register a timer event source. This is used as a watchdog timer.
	//
	watchdogSource = IOTimerEventSource::timerEventSource(this, &IntelE1000e::timeoutHandler );
	if (!watchdogSource || (myWorkLoop->addEventSource(watchdogSource) != kIOReturnSuccess)) {
		e_debug("watchdogSource create failed.");
		goto FAILED_OPENED;
	}
	
	//rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification((MAX_RX_SIZE-0x12), 1);
	rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(MAX_RX_SIZE, 1);
	txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(MAX_TX_SIZE, TBDS_PER_TCB);
	if (rxMbufCursor == NULL || rxMbufCursor == NULL) {
		e_debug("create IOMbufLittleMemory failed in start.");
		goto FAILED_OPENED;
	}
	
	pciDevice->close(this);
	e_debug("start success.");
	return true;
	
FAILED_OPENED:
	pciDevice->close(this);
FAILED_STARTED:
	super::stop(provider);
	return false;
}


IOReturn IntelE1000e::enable(IONetworkInterface * netif)
{
	e_debug("starting IntelE1000e::enable");
	int err;
	int detect_times;
	bool link_ok = false;;
	
	if (enabledForNetif) {
		e_debug("Netif has been enabled already.");
		return kIOReturnSuccess;
	}
	
	/* allocate transmit descriptors */
	err = e1000_setup_ring_resources(&adapter.tx_ring);
	if (err) {
		e_debug("e1000 setup tx resources failed.");
		goto err_setup_tx;
	}
	/* allocate receive descriptors */
	err = e1000_setup_ring_resources(&adapter.rx_ring);
	if (err) {
		e_debug("e1000 setup rx resources failed.");
		goto err_setup_rx;
	}
	
	if(adapter.hw.phy.ops.power_up)
		adapter.hw.phy.ops.power_up(&adapter.hw);
	adapter.hw.mac.ops.setup_link(&adapter.hw);
	//e1000_power_up_phy(&adapter.hw);
	
	adapter.mng_vlan_id = E1000_MNG_VLAN_NONE;
	
	/*
	 * before we allocate an interrupt, we must be ready to handle it.
	 * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
	 * as soon as we call pci_request_irq, so we have to setup our
	 * clean_rx handler before we do so.
	 */
	//e1000_configure();
	e1000_set_multi(NULL, 0);
	e1000_configure_tx();
	e1000_setup_rctl();
	e1000_configure_rx();
	e1000_alloc_rx_buffers();
	
	
	// Create and register an interrupt event source. The provider will
	// take care of the low-level interrupt registration stuff.
	//
	// enable interrupt
	interruptEnabled = true;
	E1000_WRITE_REG(&adapter.hw, E1000_IMS, IMS_ENABLE_MASK);
	//e1000_irq_enable(adapter);
	
	if (watchdogSource) {
		watchdogSource->setTimeoutMS(1000);
	} else {
		e_debug("enable watchdog source failed.");
	}
	
	if (transmitQueue) {
		transmitQueue->setCapacity(256);
		transmitQueue->start();
	} else {
		e_debug("tansmitqueue is null when enable ethernet.");
	}
	
	adapter.hw.mac.get_link_status = true;
	
	detect_times = 0;
	while(1) {
		link_ok = e1000_has_link();
		if (link_ok) {
			e_debug("about to get_link_up_info");
			adapter.hw.mac.ops.get_link_up_info(&adapter.hw,
												&adapter.link_speed,
												&adapter.link_duplex);
			e1000_print_link_info();
			setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium());
			break;
		}
		if (detect_times++ > 1000) {
			break;
		}
		IODelay(10);
	}
	
	if (link_ok) {
		UInt32 tctl;
		tctl = E1000_READ_REG(&adapter.hw, E1000_TCTL);
		tctl |= E1000_TCTL_EN;
		E1000_WRITE_REG(&adapter.hw, E1000_TCTL, tctl);
		e1000e_enable_receives();
	}
	
	if (selectMedium(getCurrentMedium()) != kIOReturnSuccess) {
		e_debug("select medium failed when driver enable.");
	}
	
	/* fire a link status change interrupt to start the watchdog */
	//E1000_WRITE_REG(&adapter.hw, E1000_ICS, E1000_ICS_LSC);
	e_debug("enable success");
	enabledForNetif = true;
	return kIOReturnSuccess; 
	
err_setup_rx:
	e1000_free_tx_resources();
err_setup_tx:
	e1000_reset();
	
	return kIOReturnIOError;
}

IOReturn IntelE1000e::disable(IONetworkInterface * netif)
{
	e_debug("IntelE1000e::disable.");
	enabledForNetif = false;
	
	struct e1000_hw* hw = &adapter.hw;
	UInt32 rctl, tctl;
	
	if (transmitQueue) {
		transmitQueue->stop();
		transmitQueue->setCapacity(0);
		transmitQueue->flush();
	}
	
	
	/* disable receives in the hardware */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	/* flush and sleep below */
	
	/* disable transmits in the hardware */
	tctl = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_EN;
	E1000_WRITE_REG(hw, E1000_TCTL, tctl);
	/* flush both disables and wait for them to finish */
	//e1e_flush();
	E1000_READ_REG(hw, E1000_STATUS);
	//msleep(10);
	IOSleep(10);
	
	/* disable irq */
	interruptEnabled = false;
	E1000_WRITE_REG(hw, E1000_IMC, ~0);
	//e1e_flush();
	E1000_READ_REG(hw, E1000_STATUS);
	
	
	//stop watchdog timer
	adapter.link_speed = 0;
	adapter.link_duplex = 0;
	
	e1000_reset();
	e1000_clean_tx_ring();
	e1000_clean_rx_ring();
	
	if (hw->phy.ops.power_down)
		hw->phy.ops.power_down(hw);
	
	//e1000_free_irq(&adapter);
	e1000_free_tx_resources();
	e1000_free_rx_resources();
	
	e1000_set_multi(NULL, 0);
	return kIOReturnSuccess;
}


UInt32 IntelE1000e::outputPacket(mbuf_t skb, void * param)
{
	
	//e_debug("IntelE1000e::outputPacket().");
	struct e1000_ring *tx_ring = &adapter.tx_ring;
	struct e1000_buffer* buffer_info = NULL;
	struct e1000_tx_desc* tx_desc = NULL;
	struct e1000_tx_desc* tx_desc_array = NULL;
	struct e1000_hw* hw = &adapter.hw;
	
	IOPhysicalSegment tx_segments[TBDS_PER_TCB];
	int count = 0;
	unsigned int i, j, first;
	UInt32 txd_upper = 0, txd_lower = 0;
	bool txCRC = false;
	
	if (enabledForNetif == false) {             // drop the packet.
		//e_debug("not enabledForNetif in outputPacket.");
		freePacket(skb);
		return kIOReturnOutputDropped;
	}
	
	if (mbuf_len(skb) <= 0) {
		//e_debug("skb <=0 in outputPacket.");
		freePacket(skb);
		return kIOReturnOutputDropped;
	}
	
	if (e1000_tx_csum(skb)) {
		txCRC = true;
	}
	txCRC = false;
	
	count = txMbufCursor->getPhysicalSegmentsWithCoalesce(skb, &tx_segments[0], TBDS_PER_TCB);
	//count = txMbufCursor->getPhysicalSegments(skb, &tx_segments[0]);
	if (count == 0) {
		//e_debug("failed to getphysicalsegment in outputPacket.");
		freePacket(skb);
		return kIOReturnOutputDropped;
	}
	netStats->outputPackets++;
	
	if (0) {
		IOLog("enter TX count:%d, len:%04x, next_to_use:%d, next_to_clean:%d, head:%d, tail:%d.\n",
		      count, mbuf_len(skb), tx_ring->next_to_use, tx_ring->next_to_clean, E1000_READ_REG(hw, tx_ring->head), E1000_READ_REG(hw, tx_ring->tail));
	}
	
	
	if (txCRC == true) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;
	}
	
	tx_desc_array = (struct e1000_tx_desc*)tx_ring->desc;
	j = first = tx_ring->next_to_use;
	
	for (i = 0; i < count; i++) {
		buffer_info = &tx_ring->buffer_info[j];
		tx_desc = &tx_desc_array[j];
		
		buffer_info->next_to_watch = j;
		buffer_info->skb = NULL;
		
		tx_desc->buffer_addr = tx_segments[i].location;
		tx_desc->lower.data = txd_lower | (tx_segments[i].length);
		tx_desc->upper.data = txd_upper;
		
		j++;
		if (j == NUM_RING_FRAME)
			j = 0;
	}
	tx_desc->lower.data |= adapter.txd_cmd;
	
	tx_ring->next_to_use = j;
	OSWriteLittleInt32((UInt8*)adapter.hw.hw_addr + tx_ring->tail, 0, j);
	OSSynchronizeIO();
	
	if (j == 0)
		j = NUM_RING_FRAME - 1;
	else
		j--;
	
	tx_ring->buffer_info[j].skb = skb;
	tx_ring->buffer_info[first].next_to_watch = j;
	IODelay(10);
	
	if (0) {
		IOLog("in TX lower:%08x, upper:%08x.\n", tx_desc->lower.data, tx_desc->upper.data);
	}
	//writel(i, adapter->hw.hw_addr + tx_ring->tail);
	
	/* Make sure there is space in the ring for the next send. */
	// e1000_maybe_stop_tx(netdev, MAX_SKB_FRAGS + 2);
	if (0) {
		IOLog("leave TX count:%d, len:%04x, next_to_use:%d, next_to_clean:%d, head:%d, tail:%d.\n",
		      count, mbuf_len(skb), tx_ring->next_to_use, tx_ring->next_to_clean, E1000_READ_REG(hw, tx_ring->head), E1000_READ_REG(hw, tx_ring->tail));
	}
	return kIOReturnOutputSuccess;
}

void IntelE1000e::getPacketBufferConstraints(IOPacketBufferConstraints * constraints) const
{
	e_debug("IntelE1000e::getPacketBufferConstraints.");
	constraints->alignStart = kIOPacketBufferAlign2;
	constraints->alignLength = kIOPacketBufferAlign1;
	return;
}

IOOutputQueue * IntelE1000e::createOutputQueue()
{
	e_debug("IntelE1000e::createOutputQueue().");
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

const OSString * IntelE1000e::newVendorString() const
{
	e_debug("const OSString * IntelE1000e::newVendorString() const.");
	return OSString::withCString("Intel");
}

const OSString * IntelE1000e::newModelString() const
{
	e_debug("const OSString * IntelE1000e::newModelString() const.");
	return OSString::withCString("E1000e");
}

IOReturn IntelE1000e::selectMedium(const IONetworkMedium * medium)
{
	e_debug("IOReturn IntelE1000e::selectMedium(const IONetworkMedium * medium).");
	bool link;
	SInt32 ret_val;
	
	ret_val = e1000_copper_link_autoneg(&adapter.hw);
	link = e1000_has_link();
	
	if (OSDynamicCast(IONetworkMedium, medium) == 0) {
		// Defaults to Auto.
		medium = mediumTable[MEDIUM_INDEX_AUTO];
	}
	
	
	if (link) {
		adapter.hw.mac.ops.get_link_up_info(&adapter.hw,
											&adapter.link_speed,
											&adapter.link_duplex);
		switch(adapter.link_speed) {
			case SPEED_1000:
				medium = mediumTable[MEDIUM_INDEX_1000FD];
				break;
			case SPEED_100:
				if (adapter.link_duplex == FULL_DUPLEX) {
					medium = mediumTable[MEDIUM_INDEX_100FD];
				} else {
					medium = mediumTable[MEDIUM_INDEX_100HD];
				}
				break;
			case SPEED_10:
				if (adapter.link_duplex == FULL_DUPLEX) {
					medium = mediumTable[MEDIUM_INDEX_10FD];
				} else {
					medium = mediumTable[MEDIUM_INDEX_10HD];
				}
				break;
			default:
				break;
		}
	}
	
	if (medium) {
		if (!setCurrentMedium(medium)) {
			e_debug("setCurrentMedium error.");
		}
	}
	
	return ( medium ? kIOReturnSuccess : kIOReturnIOError );
}

bool IntelE1000e::configureInterface(IONetworkInterface * interface)
{
	e_debug("IntelE1000e::configureInterface.");
	
	IONetworkData * data = NULL;
	
	if (super::configureInterface(interface) == false) {
		e_debug("IOEthernetController::confiugureInterface failed."); 
		return false;
	}
	
	// Get the generic network statistics structure.
	data = interface->getParameter(kIONetworkStatsKey);
	if (!data || !(netStats = (IONetworkStats *) data->getBuffer())) {
		e_debug("netif getParameter NetworkStatsKey failed."); 
		return false;
	}
	
	// Get the Ethernet statistics structure.
	
	data = interface->getParameter(kIOEthernetStatsKey);
	if (!data || !(etherStats = (IOEthernetStats *) data->getBuffer())) {
		e_debug("netif getParameter kIOEthernetStatsKey failed."); 
		return false;
	}
	
	return true;
}

bool IntelE1000e::createWorkLoop()
{
	e_debug("IntelE1000e::createWorkLoop().");
	workLoop = IOWorkLoop::workLoop();	
	return (workLoop !=  NULL);
}

IOWorkLoop * IntelE1000e::getWorkLoop() const
{
	e_debug("IntelE1000e::getWorkLoop() const.");
	return workLoop;
}

//-----------------------------------------------------------------------
// Methods inherited from IOEthernetController.
//-----------------------------------------------------------------------

IOReturn IntelE1000e::getHardwareAddress(IOEthernetAddress * addr)
{
	e_debug("IntelE1000e::getHardwareAddress(IOEthernetAddress * addr).");
	memcpy(addr, adapter.hw.mac.addr, kIOEthernetAddressSize);
	return kIOReturnSuccess;
}
IOReturn IntelE1000e::setHardwareAddress(const IOEthernetAddress * addr)
{
	e_debug("IntelE1000e::setHardwareAddress(const IOEthernetAddress * addr).");
	return kIOReturnSuccess;
}
IOReturn IntelE1000e::setPromiscuousMode(bool active)
{
	e_debug("IntelE1000e::setPromiscuousMode(bool active).");
	return kIOReturnSuccess;
}
IOReturn IntelE1000e::setMulticastMode(bool active)
{
	e_debug("IntelE1000e::setMulticastMode(bool active).");
	
	return kIOReturnSuccess;
}
IOReturn IntelE1000e::setMulticastList(IOEthernetAddress * addrs, UInt32 count)
{
	//e_debug("IntelE1000e::setMulticastList(IOEthernetAddress * addrs, UInt32 count).");
	if (e1000_set_multi(addrs, count) == false) {
		return kIOReturnError;
	}
	return kIOReturnSuccess;
}

IOReturn IntelE1000e::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) 
{
	if( checksumFamily != kChecksumFamilyTCPIP ) {
		IOLog("IntelE1000e: Operating system wants information for unknown checksum family.\n");
		return kIOReturnUnsupported;
	} else {
		if( !isOutput ) {
			*checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;
		} else {
			//*checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;
			*checksumMask = 0;
		}
		return kIOReturnSuccess;
	}
}

//-----------------------------------------------------------------------
// e1000e private functions
//-----------------------------------------------------------------------
bool IntelE1000e::addNetworkMedium(UInt32 type, UInt32 bps, UInt32 index)
{
	IONetworkMedium *medium;
	
	medium = IONetworkMedium::medium(type, bps, 0, index);
	if (!medium) {
		e_debug("Couldn't allocate medium.");
		return false;
	}
	
	if (!IONetworkMedium::addMedium(mediumDict, medium)) {
		e_debug("Couldn't add medium.");
		return false;
	}
	
	mediumTable[index] = medium;
	return true;
}

void IntelE1000e::interruptOccurred(IOInterruptEventSource * src)
{
	int i;
	struct e1000_hw* hw = &adapter.hw;
	
	if (interruptReason & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		hw->mac.get_link_status = true;
		/*
		 * ICH8 workaround-- Call gig speed drop workaround on cable
		 * disconnect (LSC) before accessing any PHY registers
		 */
		
		if ((adapter.flags & FLAG_LSC_GIG_SPEED_DROP) &&
			(!(E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_LU)))
			e1000e_gig_downshift_workaround_ich8lan(hw);
		
	}
	
	for (i = 0; i < E1000_MAX_INTR; i++) {
		int rx_cleaned;
		rx_cleaned = e1000_clean_rx_irq();
		e1000_clean_tx_irq();
		
		if (!rx_cleaned)
			break;
	}
	
	return;
}

void IntelE1000e::interruptHandler(OSObject * target, IOInterruptEventSource * src, int count)
{
	IntelE1000e * me = (IntelE1000e *) target;
	me->interruptOccurred(src);
	return;
}

bool IntelE1000e::interruptFilter(OSObject * target, IOFilterInterruptEventSource * src )
{
	IntelE1000e * me = (IntelE1000e *) target;
	struct e1000_hw* hw = &me->adapter.hw;
	bool bret = false;
	UInt32 icr;
	
	if (me->interruptEnabled) {
		icr = E1000_READ_REG(hw, E1000_ICR);
		if (icr & 0x000FFFFF) {
			me->interruptReason = icr;
			bret = true;
		}
	}	
	return bret;
}


SInt32 IntelE1000e::e1000_read_pcie_cap_reg(struct e1000_hw *hw, UInt32 reg, UInt16 *value)
{
	e_debug("IntelE1000e::e1000_read_pcie_cap_reg(struct e1000_hw *hw, UInt32 reg, UInt16 *value)");
	
	UInt8 cap_offset;
	
	if (pciDevice == NULL) 
		return -E1000_ERR_CONFIG;
	
	pciDevice->findPCICapability(kIOPCIPCIExpressCapability, &cap_offset);
	if (!cap_offset)
		return -E1000_ERR_CONFIG;
	
	*value = pciDevice->configRead16(cap_offset + reg);
	
	return E1000_SUCCESS;
}

/**
 * e1000e_setup_tx_resources - allocate Tx resources (Descriptors)
 * @adapter: board private structure
 *
 * Return 0 on success, negative on failure
 **/
SInt32 IntelE1000e::e1000_setup_ring_resources(struct e1000_ring* ring)
{
	void *desc;
	IOPhysicalAddress dma;
	
	bzero(ring, sizeof(*ring));
	
	desc = IOMallocContiguous(SIZE_RING_DESC, PAGE_SIZE, &dma);
	if (desc == NULL) {
		e_debug("No memory for ring desc.");
		return -1;
	}
	
	bzero(desc, SIZE_RING_DESC);
	ring->dma = dma;
	ring->desc = desc;
	
	e_debug("ring desc success: size->%d, virtual:%08x, phy:%08x.", SIZE_RING_DESC, desc, dma);
	
	return 0;
}

/**
 * e1000_configure_tx - Configure 8254x Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
void IntelE1000e::e1000_configure_tx()
{
	struct e1000_hw *hw = &adapter.hw;
	struct e1000_ring *tx_ring = &adapter.tx_ring;
	UInt32 tdlen, tctl, tipg;
	UInt32 ipgr1, ipgr2;
	
	/* Setup the HW Tx Head and Tail descriptor pointers */
	tdlen = NUM_RING_FRAME * sizeof(struct e1000_tx_desc);
	
	e_debug("configure tx tdlen->%d", tdlen);
	E1000_WRITE_REG(hw, E1000_TDBAL(0), tx_ring->dma);
	E1000_WRITE_REG(hw, E1000_TDBAH(0), 0);
	E1000_WRITE_REG(hw, E1000_TDLEN(0), tdlen);
	E1000_WRITE_REG(hw, E1000_TDH(0), 0);
	E1000_WRITE_REG(hw, E1000_TDT(0), 0);
	tx_ring->head = E1000_TDH(0);
	tx_ring->tail = E1000_TDT(0);
	
	/* Set the default values for the Tx Inter Packet Gap timer */
	tipg = DEFAULT_82543_TIPG_IPGT_COPPER;          /*  8  */
	ipgr1 = DEFAULT_82543_TIPG_IPGR1;               /*  8  */
	ipgr2 = DEFAULT_82543_TIPG_IPGR2;               /*  6  */
	
	tipg |= ipgr1 << E1000_TIPG_IPGR1_SHIFT;
	tipg |= ipgr2 << E1000_TIPG_IPGR2_SHIFT;
	E1000_WRITE_REG(hw, E1000_TIPG, tipg);
	
	/* Set the Tx Interrupt Delay register */
	//E1000_WRITE_REG(hw, E1000_TIDV, adapter.tx_int_delay);
	/* Tx irq moderation */
	//E1000_WRITE_REG(hw, E1000_TADV, adapter.tx_abs_int_delay);
	
	/* Program the Transmit Control Register */
	tctl = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= E1000_TCTL_PSP | E1000_TCTL_RTLC |
	(E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);
	
	hw->mac.ops.config_collision_dist(hw);
	
	/* Setup Transmit Descriptor Settings for eop descriptor */
	adapter.txd_cmd = E1000_TXD_CMD_EOP | E1000_TXD_CMD_IFCS;
	
	/* only set IDE if we are delaying interrupts using the timers */
	//if (adapter.tx_int_delay)
	//adapter.txd_cmd |= E1000_TXD_CMD_IDE;
	
	/* enable Report Status bit */
	adapter.txd_cmd |= E1000_TXD_CMD_RS;
	
	E1000_WRITE_REG(hw, E1000_TCTL, tctl);
	
	//adapter.tx_queue_len = adapter.netdev->tx_queue_len;
}

/**
 * e1000_setup_rctl - configure the receive control registers
 * @adapter: Board private structure
 **/
void IntelE1000e::e1000_setup_rctl()
{
	struct e1000_hw *hw = &adapter.hw;
	UInt32 rctl;
	
	rctl = 0x04008002;	
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);
	/* just started the receive unit, no need to restart */
	adapter.flags &= ~FLAG_RX_RESTART_NOW;
}

/**
 * e1000_configure_rx - Configure Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
void IntelE1000e::e1000_configure_rx()
{
	struct e1000_hw *hw = &adapter.hw;
	struct e1000_ring *rx_ring = &adapter.rx_ring;
	UInt32 rdlen, rctl, rxcsum, ctrl_ext;
	
	rdlen = NUM_RING_FRAME * sizeof(struct e1000_rx_desc);
	
	e_debug("configure rx rdlen->%d", rdlen);
	/* disable receives while setting up the descriptors */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	E1000_READ_REG(hw, E1000_STATUS);
	//	e1e_flush();
	IODelay(10);
	
	/* set the Receive Delay Timer Register */
	//E1000_WRITE_REG(hw, E1000_RDTR, adapter.rx_int_delay);
	
	/* irq moderation */
	//E1000_WRITE_REG(hw, E1000_RADV, adapter.rx_abs_int_delay);
	
	ctrl_ext = E1000_READ_REG(hw, E1000_CTRL_EXT);
	/* Reset delay timers after every interrupt */
	ctrl_ext |= E1000_CTRL_EXT_INT_TIMER_CLR;
	
	E1000_WRITE_REG(hw, E1000_CTRL_EXT, ctrl_ext);
	E1000_READ_REG(hw, E1000_STATUS);
	//	e1e_flush();
	IODelay(10);
	
	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring
	 */
	E1000_WRITE_REG(hw, E1000_RDBAL(0), rx_ring->dma);
	E1000_WRITE_REG(hw, E1000_RDBAH(0), 0);
	E1000_WRITE_REG(hw, E1000_RDLEN(0), rdlen);
	E1000_WRITE_REG(hw, E1000_RDH(0), 0);
	E1000_WRITE_REG(hw, E1000_RDT(0), 0);
	rx_ring->head = E1000_RDH(0);
	rx_ring->tail = E1000_RDT(0);
	
	/* Enable Receive Checksum Offload for TCP and UDP */
	rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
	rxcsum |= E1000_RXCSUM_TUOFL;
	E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);
	
	/* Enable Receives */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);
	E1000_READ_REG(hw, E1000_STATUS);
	IODelay(10);
}

/**
 * e1000_desc_unused - calculate if we have unused descriptors
 **/
int IntelE1000e::e1000_desc_unused(struct e1000_ring *ring)
{
	if (ring->next_to_clean > ring->next_to_use)
		return ring->next_to_clean - ring->next_to_use - 1;
	
	return NUM_RING_FRAME + ring->next_to_clean - ring->next_to_use - 1;
}


/**
 * e1000_alloc_rx_buffers - Replace used receive buffers; legacy & extended
 * @adapter: address of board private structure
 **/
void IntelE1000e::e1000_alloc_rx_buffers()
{
	struct e1000_ring *rx_ring = &adapter.rx_ring;
	struct e1000_rx_desc *rx_desc, *rx_desc_array;
	struct e1000_buffer *buffer_info;
	struct IOPhysicalSegment vector;
	
	mbuf_t skb;
	UInt32 count;
	
	UInt32 i;
	//UInt32 bufsz = kIOEthernetMaxPacketSize;
	UInt32 bufsz = MAX_RX_SIZE;
	
	e_debug("starting e1000_alloc_rx_buffers");
	
	rx_desc_array = (struct e1000_rx_desc*)rx_ring->desc;
	
	for (i = 0; i < NUM_RING_FRAME; i++) {
		buffer_info = &rx_ring->buffer_info[i];
		rx_desc = &rx_desc_array[i];
		
		//kIOEthernetMaxPacketSize
		skb = allocatePacket(bufsz);
		if (skb == NULL) {
			IOLog("allocatePacket failed in alloc_rx_buffer.\n");
			break;
		}
		
		//mbuf_adj(skb, 0x12);
		count = rxMbufCursor->getPhysicalSegmentsWithCoalesce(skb, &vector, 1);
		if (count == 0) {
			IOLog("getPhysicalSegments failed in alloc_rx_buffer.\n");
			freePacket(skb);
			break;
		}
		buffer_info->skb = skb;
		rx_desc->buffer_addr = vector.location;
		//IOLog("index:%d, addr:%08x.\n", i, vector.location);
	}
	
	rx_ring->next_to_use = NUM_RING_FRAME - 1;
	E1000_WRITE_REG(&adapter.hw, rx_ring->tail, NUM_RING_FRAME - 2);
}


bool IntelE1000e::e1000_clean_rx_irq()
{
	struct e1000_ring *rx_ring = &adapter.rx_ring;
	struct e1000_rx_desc *rx_desc, *next_rxd, *rx_desc_array;
	struct e1000_buffer *buffer_info, *next_buffer;
	struct e1000_hw* hw = &adapter.hw;
	
	UInt32 length;
	UInt32 head, tail;
	unsigned int i;
	bool cleaned = 0;
	
	
	i = rx_ring->next_to_clean % NUM_RING_FRAME;
	rx_desc_array = (struct e1000_rx_desc*)rx_ring->desc;
	rx_desc = &rx_desc_array[i];
	buffer_info = &rx_ring->buffer_info[i];
	head = E1000_READ_REG(hw, rx_ring->head);
	tail = E1000_READ_REG(hw, rx_ring->tail);
	
	/*
	 IOLog("crl_ext:%08x, rxcsum:%08x, rxdctl:%08x, rctl:%08x.\n",
	 E1000_READ_REG(hw, E1000_CTRL_EXT),
	 E1000_READ_REG(hw, E1000_RXCSUM),
	 E1000_READ_REG(hw, E1000_RXDCTL(0)),
	 E1000_READ_REG(hw, E1000_RCTL));
	 IOLog("enter RX head:%d, tail:%d, status:%02x, len:%04x. \n",
	 head, tail, rx_desc->status, rx_desc->length);
	 if ((head - i > 1) && 0) {
	 IOLog("ENTER RX .....\n\n");
	 int kk;
	 struct e1000_rx_desc* rd;
	 
	 for (kk = 0; kk < NUM_RING_FRAME; kk++) {
	 rd = E1000_RX_DESC(*rx_ring, kk);
	 if (rd->length || rd->status || rd->errors) {
	 IOLog("---[rd:%p index:%d, status:%02x, errors:%02x, len:%04x].\n",
	 rd, kk, rd->status, rd->errors, rd->length);
	 }
	 }
	 IOLog("\n");
	 */
	if (0)
		IOLog("enter RX next_to_clean:%d next_to_use:%d, head:%d, tail:%d, status:%02x, len:%04x, err:%02x. \n",
			  rx_ring->next_to_clean, rx_ring->next_to_use, head, tail, rx_desc->status, rx_desc->length, rx_desc->errors);
	while ((rx_desc->status & E1000_RXD_STAT_DD)) {
		mbuf_t skb;
		UInt8 status;
		UInt32 ckResult;
		struct IOPhysicalSegment vector;
		
		status = rx_desc->status;
		
		if (0)
			IOLog("index->%d, status->0x%0X, length->%04X, errors->%02X, head:%d, tail:%d, skb:%08x, addr:%08x.\n",
				  i, rx_desc->status, OSSwapLittleToHostInt16(rx_desc->length), rx_desc->errors, head, tail, buffer_info->skb, rx_desc->buffer_addr);
		i++;
		if (i == NUM_RING_FRAME)
			i = 0;
		
		next_rxd = &rx_desc_array[i];
		next_buffer = &rx_ring->buffer_info[i];
		
		cleaned = 1;
		length = OSSwapLittleToHostInt16(rx_desc->length);
		/*
		 if (length > 32) {
		 unsigned char* data = (unsigned char*)mbuf_data(buffer_info->skb);
		 IOLog("ip_len:%02x%02x, proto:%02x\n", data[16], data[17], data[23]);
		 }
		 */
		if (rx_desc->errors) 
			goto next_desc;
		
		/* !EOP means multiple descriptors were used to store a single
		 * packet, also make sure the frame isn't just CRC only */
		if (!(status & E1000_RXD_STAT_EOP) || (length <= 4)) {
			/* All receives must fit into a single buffer */
			IOLog("Receive packet consumed multiple buffers\n");
			goto next_desc;
		}
		
		/*
		 if (length > 0x500) {
		 goto next_desc;
		 }
		 */
		
		//skb = copyPacket(buffer_info->skb, length);
		skb = buffer_info->skb;
		
		ckResult = 0;
		if (status & E1000_RXD_STAT_IPCS) {
			ckResult |= kChecksumIP;
		}
		if (status & E1000_RXD_STAT_UDPCS) {
			ckResult |= kChecksumUDP;
		}
		if (status & E1000_RXD_STAT_TCPCS) {
			ckResult |= kChecksumTCP;
		} 
		setChecksumResult(skb, kChecksumFamilyTCPIP, kChecksumIP | kChecksumTCP | kChecksumUDP, ckResult);
		netif->inputPacket(skb, length);
		netStats->inputPackets++;
		buffer_info->skb = allocatePacket(MAX_RX_SIZE);
		//mbuf_adj(buffer_info->skb, 0x12);
		rxMbufCursor->getPhysicalSegmentsWithCoalesce(buffer_info->skb, &vector, 1);
		rx_desc->buffer_addr = vector.location;
		
	next_desc:
		rx_desc->status = 0;
		rx_desc->errors = 0;
		rx_desc->length = 0;
		
		// alloc
		rx_ring->next_to_use++;
		if (rx_ring->next_to_use == NUM_RING_FRAME) {
			rx_ring->next_to_use = 0; 
		}
		
		if (rx_ring->next_to_use == 0) {
			E1000_WRITE_REG(hw, rx_ring->tail, NUM_RING_FRAME - 1);
		} else {
			E1000_WRITE_REG(hw, rx_ring->tail, rx_ring->next_to_use - 1);
		}
		// alloc end
		
		/* use prefetched values */
		rx_desc = next_rxd;
		buffer_info = next_buffer;
	}
	rx_ring->next_to_clean = i;
	
	/*
	 if (cleaned)
	 netif->flushInputQueue();
	 */
	/*
	 if (0) {
	 IOLog("DDING rx cleaned:%d, next_to_clean->%d, rx_packets->%d, rx_bytes->%d, total_packets->%d, total_bytes->%d.\n", cleaned, i, total_rx_packets, total_rx_bytes, adapter.total_rx_packets, adapter.total_rx_bytes);
	 }
	 if (0) {
	 IOLog("leave RX next_to_clean:%d next_to_use:%d, head:%d, tail:%d. \n\n",
	 rx_ring->next_to_clean, rx_ring->next_to_use, E1000_READ_REG(hw, rx_ring->head), E1000_READ_REG(hw, rx_ring->tail));
	 }
	 */      
	
	return cleaned;
}


/**
 * e1000_clean_tx_irq - Reclaim resources after transmit completes
 * @adapter: board private structure
 *
 * the return value indicates if there is more work to do (later)
 **/
bool IntelE1000e::e1000_clean_tx_irq()
{
	struct e1000_ring *tx_ring = &adapter.tx_ring;
	struct e1000_tx_desc *tx_desc, *eop_desc, *tx_desc_array;
	struct e1000_buffer *buffer_info;
	unsigned int i, eop;
	bool cleaned = 0, retval = 1;
	bool do_service = false;
	
	i = tx_ring->next_to_clean;
	eop = tx_ring->buffer_info[i].next_to_watch;
	tx_desc_array = (struct e1000_tx_desc*) tx_ring->desc;
	eop_desc = &tx_desc_array[eop];
	
	
	if (0) {		
		IOLog("DDING: clean_tx_irq: next_to_clean->%d, eop->%d.\n", i, eop);
	}
	
	while (eop_desc->upper.data & (E1000_TXD_STAT_DD)) {
		for (cleaned = 0; !cleaned; ) {
			tx_desc = &tx_desc_array[i];
			buffer_info = &tx_ring->buffer_info[i];
			
			cleaned = (i == eop);
			
			if (cleaned) {
				do_service = true;
			}
			
			if (buffer_info->skb) {
				freePacket(buffer_info->skb);
				buffer_info->skb = NULL;
			}
			
			tx_desc->upper.data = 0;
			
			i++;
			if (i == NUM_RING_FRAME)
				i = 0;
		}
		
		eop = tx_ring->buffer_info[i].next_to_watch;
		eop_desc = &tx_desc_array[eop];
	}
	
	tx_ring->next_to_clean = i;
	
	
	if (do_service) {
		transmitQueue->service();
	}
	
	return retval;
}


void IntelE1000e::e1000e_enable_receives()
{
	/* make sure the receive unit is started */
	struct e1000_hw *hw = &adapter.hw;
	UInt32 rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl | E1000_RCTL_EN);
}

bool IntelE1000e::e1000_has_link()
{
	struct e1000_hw *hw = &adapter.hw;
	bool link_active = 0;
	SInt32 ret_val = 0;
	
	/*
	 * get_link_status is set on LSC (link status) interrupt or
	 * Rx sequence error interrupt.  get_link_status will stay
	 * false until the check_for_link establishes link
	 * for copper adapters ONLY
	 */
	
	if (hw->mac.get_link_status) {
		ret_val = hw->mac.ops.check_for_link(hw);
		link_active = !hw->mac.get_link_status;
	} else {
		link_active = 1;
	}
	
	if ((ret_val == E1000_ERR_PHY) && (hw->phy.type == e1000_phy_igp_3) &&
		(E1000_READ_REG(hw, E1000_CTRL) & E1000_PHY_CTRL_GBE_DISABLE)) {
		/* See e1000_kmrn_lock_loss_workaround_ich8lan() */
		e_debug("Gigabit has been disabled, downgrading speed");
	}
	
	return link_active;
}


/**
 * e1000e_reset - bring the hardware into a known good state
 *
 * This function boots the hardware and enables some settings that
 * require a configuration cycle of the hardware - those cannot be
 * set/changed during runtime. After reset the device needs to be
 * properly configured for Rx, Tx etc.
 */
void IntelE1000e::e1000_reset()
{
	struct e1000_mac_info *mac = &adapter.hw.mac;
	struct e1000_fc_info *fc = &adapter.hw.fc;
	struct e1000_hw *hw = &adapter.hw;
  	UInt16 phy_data = 0;
	
	/* reset Packet Buffer Allocation to default */
	//E1000_WRITE_REG(hw, E1000_PBA, pba);
	if (adapter.ei->mac == e1000_ich8lan) {
		E1000_WRITE_REG(hw, E1000_PBA, 8);
		E1000_WRITE_REG(hw, E1000_PBS, 16);
	} else {
		E1000_WRITE_REG(hw, E1000_PBA, adapter.ei->pba);
	}
	
	fc->high_water = 6672;
	fc->low_water = fc->high_water - 8;
	fc->pause_time = E1000_FC_PAUSE_TIME;
	fc->send_xon = 1;
	fc->type = fc->original_type;
	
	/* Allow time for pending master requests to run */
	mac->ops.reset_hw(hw);
	
 	/* Enable h/w to recognize an 802.1Q VLAN Ethernet packet */
	//E1000_WRITE_REG(hw, E1000_VET, ETH_P_8021Q);
	E1000_WRITE_REG(hw, E1000_VET, 0x00008100);
	
	e1000e_reset_adaptive(hw);
	
	E1000_WRITE_REG(hw, E1000_WUC, 0);
	
	mac->ops.init_hw(hw);
	
	if (!hw->phy.ops.get_info) {
		return;
	}
	
	hw->phy.ops.get_info(hw);
	
	hw->phy.ops.read_reg(hw, IGP02E1000_PHY_POWER_MGMT, &phy_data);
	phy_data &= ~IGP02E1000_PM_SPD;
	hw->phy.ops.write_reg(hw, IGP02E1000_PHY_POWER_MGMT, phy_data);
	
	return;
}


/**
 * e1000_clean_tx_ring - Free Tx Buffers
 * @adapter: board private structure
 **/
void IntelE1000e::e1000_clean_tx_ring()
{
	struct e1000_ring *ring = &adapter.tx_ring;
	struct e1000_hw* hw = &adapter.hw;
	unsigned int i;
	
	for (i = 0; i < NUM_RING_FRAME; i++) {
		bzero(&ring->buffer_info[i], sizeof(struct e1000_buffer));
	}
	
	if (ring->desc) {
		bzero(ring->desc, SIZE_RING_DESC);
	}
	
	ring->next_to_use = 0;
	ring->next_to_clean = 0;
	
	E1000_WRITE_REG(hw, ring->head, 0);
	E1000_WRITE_REG(hw, ring->tail, 0);
	
	//writel(0, adapter.hw.hw_addr + tx_ring->head);
	//writel(0, adapter.hw.hw_addr + tx_ring->tail);
}


/**
 * e1000_free_tx_resources - Free Tx Resources per Queue
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
void IntelE1000e::e1000_free_tx_resources()
{
	struct e1000_ring *ring = &adapter.tx_ring;
	
	if (ring->desc) {
		IOFreeContiguous(ring->desc, SIZE_RING_DESC);
		ring->desc = NULL;
	}
	bzero(ring, sizeof(*ring));
}


/**
 * e1000_clean_rx_ring - Free Rx Buffers per Queue
 * @adapter: board private structure
 **/
void IntelE1000e::e1000_clean_rx_ring()
{
	struct e1000_ring *ring = &adapter.rx_ring;
	struct e1000_hw* hw = &adapter.hw;
	unsigned int i;
	
	for (i = 0; i < NUM_RING_FRAME; i++) {
		if (ring->buffer_info[i].skb != NULL) {
			freePacket(ring->buffer_info[i].skb);
			ring->buffer_info[i].skb = NULL;
		}
	}
	
	if (ring->desc) {
		bzero(ring->desc, SIZE_RING_DESC);
	}
	
	ring->next_to_use = 0;
	ring->next_to_clean = 0;
	
	E1000_WRITE_REG(hw, ring->head, 0);
	E1000_WRITE_REG(hw, ring->tail, 0);
	
	//writel(0, adapter.hw.hw_addr + rx_ring->head);
	//writel(0, adapter.hw.hw_addr + rx_ring->tail);
}


/**
 * e1000_free_rx_resources - Free Rx Resources
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/

void IntelE1000e::e1000_free_rx_resources()
{
	struct e1000_ring *ring = &adapter.rx_ring;
	
	if (ring->desc) {
		IOFreeContiguous(ring->desc, SIZE_RING_DESC);
		ring->desc = NULL;
	}
	bzero(ring, sizeof(*ring));
}

void IntelE1000e::timeoutOccurred(IOTimerEventSource* src)
{
	watchdogSource->setTimeoutMS(3000);
	
	struct e1000_mac_info *mac = &adapter.hw.mac;
	struct e1000_hw *hw = &adapter.hw;
	UInt32 link, tctl;
	
	link = e1000_has_link();
	
	//e_debug("timeout link:%d, preLinkStatus:%d.", link, preLinkStatus);
	if (link && link == preLinkStatus) {
		//e1000e_enable_receives();
		goto link_up;
	}
	
	if (link) {
		mac->ops.get_link_up_info(&adapter.hw,
								  &adapter.link_speed,
								  &adapter.link_duplex);
		e1000_print_link_info();
		/*
		 * On supported PHYs, check for duplex mismatch only
		 * if link has autonegotiated at 10/100 half
		 */
		if ((hw->phy.type == e1000_phy_igp_3 ||
		     hw->phy.type == e1000_phy_bm) &&
		    (hw->mac.autoneg == true) &&
		    (adapter.link_speed == SPEED_10 ||
		     adapter.link_speed == SPEED_100) &&
		    (adapter.link_duplex == HALF_DUPLEX)) {
			UInt16 autoneg_exp;
			
			hw->phy.ops.read_reg(hw, PHY_AUTONEG_EXP,
								 &autoneg_exp);
			
			if (!(autoneg_exp & NWAY_ER_LP_NWAY_CAPS))
				e_info("Autonegotiated half duplex but"
				       " link partner cannot autoneg. "
				       " Try forcing full duplex if "
				       "link gets many collisions.\n");
		}
		
		/*
		 * enable transmits in the hardware, need to do this
		 * after setting TARC(0)
		 */
		tctl = E1000_READ_REG(hw, E1000_TCTL);
		tctl |= E1000_TCTL_EN;
		E1000_WRITE_REG(hw, E1000_TCTL, tctl);
		e1000e_enable_receives();
		E1000_READ_REG(hw, E1000_STATUS);
		setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium());
		if (transmitQueue) {
			transmitQueue->start();
		}
		
	} else {
		
		if (link != preLinkStatus) {
			adapter.link_speed = 0;
			adapter.link_duplex = 0;
			e_info("Link is Down\n");
			
			transmitQueue->stop();
			setLinkStatus(kIONetworkLinkValid, 0);
			
		}
	}
	
link_up:
	/* Cause software interrupt to ensure Rx ring is cleaned */
	//E1000_WRITE_REG(hw, E1000_ICS, E1000_ICS_RXDMT0);
	
	preLinkStatus = link;
}

void IntelE1000e::timeoutHandler(OSObject * target, IOTimerEventSource * src)
{
	//e_debug("void IntelE1000e::timeoutHandler(OSObject * target, IOTimerEventSource * src)");
	IntelE1000e* me = (IntelE1000e*) target;
	me->timeoutOccurred(src);
	
}

void IntelE1000e::e1000_print_link_info()
{
	struct e1000_hw *hw = &adapter.hw;
	UInt32 ctrl = E1000_READ_REG(hw, E1000_CTRL);
	
	e_info("Link is Up %d Mbps %s, Flow Control: %s\n",
		   adapter.link_speed,
		   (adapter.link_duplex == FULL_DUPLEX) ?
	       "Full Duplex" : "Half Duplex",
		   ((ctrl & E1000_CTRL_TFCE) && (ctrl & E1000_CTRL_RFCE)) ?
	       "RX/TX" :
		   ((ctrl & E1000_CTRL_RFCE) ? "RX" :
			((ctrl & E1000_CTRL_TFCE) ? "TX" : "None" )));
}

bool IntelE1000e::e1000_set_multi(IOEthernetAddress * addrs, UInt32 count)
{
	struct e1000_hw *hw = &adapter.hw;
//	struct e1000_mac_info *mac = &hw->mac;
	IOBufferMemoryDescriptor *mta_desc;
	unsigned char* mta_list;
	UInt32 rctl;
	int i;
	
	/* Check for Promiscuous and All Multicast modes */
	
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	
	if (count) {
		
		mta_desc = IOBufferMemoryDescriptor::withOptions(kIOMemoryUnshared, PAGE_SIZE, PAGE_SIZE);
		if (mta_desc == NULL)
			return false;
		
		mta_list = (unsigned char*)mta_desc->getBytesNoCopy();
		
		for (i = 0; i < count; i++) {
			memcpy(mta_list + (i*kIOEthernetAddressSize), addrs + i, kIOEthernetAddressSize);
		}
		
		hw->mac.ops.update_mc_addr_list(hw, mta_list, i);
		mta_desc->release();
	} else {
		/*
		 * if we're called from probe, we might not have
		 * anything to do here, so clear out the list
		 */
		rctl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE);
		E1000_WRITE_REG(hw, E1000_RCTL, rctl);
		hw->mac.ops.update_mc_addr_list(hw, NULL, 0);
	}
	return true;
}

bool IntelE1000e::e1000_tx_csum(mbuf_t skb)
{
	/*
	 struct e1000_ring *tx_ring = &adapter.tx_ring;
	 struct e1000_context_desc *context_desc;
	 struct e1000_buffer *buffer_info;
	 unsigned int i;
	 UInt8 css;
	 UInt32 cmd_len = E1000_TXD_CMD_DEXT;
	 UInt32 checksumDemanded;
	 
	 getChecksumDemand(buf, kChecksumFamilyTCPIP, &checksumDemanded);
	 
	 if(checksumDemanded == 0)
	 return false;
	 
	 if( (checksumDemanded & kChecksumTCP) || (checksumDemanded & kChecksumUDP) )
	 cmd_len |= E1000_TXD_CMD_TCP;
	 
	 css = skb_transport_offset(skb);
	 
	 i = tx_ring->next_to_use;
	 buffer_info = &tx_ring->buffer_info[i];
	 context_desc = E1000_CONTEXT_DESC(*tx_ring, i);
	 
	 context_desc->lower_setup.ip_config = 0;
	 context_desc->upper_setup.tcp_fields.tucss = css;
	 context_desc->upper_setup.tcp_fields.tucso = css + skb->csum_offset;
	 context_desc->upper_setup.tcp_fields.tucse = 0;
	 context_desc->tcp_seg_setup.data = 0;
	 context_desc->cmd_and_length = cmd_len;
	 buffer_info->skb = NULL;
	 buffer_info->next_to_watch = i;
	 
	 i++;
	 if (i == tx_ring->count)
	 i = 0;
	 tx_ring->next_to_use = i;
	 
	 return true;
	 */
	return false;
}

IOReturn IntelE1000e::registerWithPolicyMaker ( IOService * policyMaker )
{
	static IOPMPowerState powerStateArray[ 2 ] = {
		{ 1,0,0,0,0,0,0,0,0,0,0,0 },
		{ 1,kIOPMDeviceUsable,kIOPMPowerOn,kIOPMPowerOn,0,0,0,0,0,0,0,0 }
	};
	
	return policyMaker->registerPowerDriver( this, powerStateArray, 2 );
}
