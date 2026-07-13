// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Linaro Ltd.
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/sizes.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/ktime.h>

#include "core.h"
#include "firmware.h"
#include "hfi_venus_io.h"

#define VENUS_PAS_ID			9
#define VENUS_FW_MEM_SIZE		(6 * SZ_1M)
#define VENUS_FW_START_ADDR		0x0

/*
 * Iris1 is still an experimental bring-up target.  Keep the diagnostic
 * stages in the Venus module so one kernel build can distinguish a CPU
 * mapping failure from an MDT/PAS load or firmware-start failure.
 *
 * 0: normal/full boot
 * 1: map and unmap the reserved region, then stop the probe
 * 2: load the MDT image (including PAS init), but do not auth/reset it
 * 3: auth/reset and immediately shut the firmware down
 * 4: auth/reset, configure secure ranges and immediately shut it down
 * 5: auth/reset, configure secure ranges, hold, then shut it down
 */
static unsigned int iris1_fw_stage;
module_param(iris1_fw_stage, uint, 0400);
MODULE_PARM_DESC(iris1_fw_stage,
		 "Iris1 firmware stage: 0=full, 1=map, 2=load, 3=auth-stop, 4=protect-stop, 5=hold-stop");

static unsigned int iris1_fw_hold_ms = 100;
module_param(iris1_fw_hold_ms, uint, 0400);
MODULE_PARM_DESC(iris1_fw_hold_ms,
		 "Iris1 stage 5 runtime before immediate firmware shutdown");

static unsigned int iris1_fw_checkpoint_ms = 1500;
module_param(iris1_fw_checkpoint_ms, uint, 0400);
MODULE_PARM_DESC(iris1_fw_checkpoint_ms,
		 "Delay after Iris1 firmware checkpoints for persistent logging");

static void venus_fw_checkpoint(struct venus_core *core, const char *stage)
{
	if (!IS_IRIS1(core))
		return;

	dev_info(core->dev, "Iris1 firmware checkpoint: %s\n", stage);
	msleep(min(iris1_fw_checkpoint_ms, 5000U));
}

static void venus_reset_cpu(struct venus_core *core)
{
	u32 fw_size = core->fw.mapped_mem_size;
	void __iomem *wrapper_base;

	if (IS_IRIS2(core) || IS_IRIS2_1(core))
		wrapper_base = core->wrapper_tz_base;
	else
		wrapper_base = core->wrapper_base;

	writel(0, wrapper_base + WRAPPER_FW_START_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_FW_END_ADDR);
	writel(0, wrapper_base + WRAPPER_CPA_START_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_CPA_END_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_NONPIX_START_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_NONPIX_END_ADDR);

	if (IS_IRIS2(core) || IS_IRIS2_1(core)) {
		/* Bring XTSS out of reset */
		writel(0, wrapper_base + WRAPPER_TZ_XTSS_SW_RESET);
	} else {
		writel(0x0, wrapper_base + WRAPPER_CPU_CGC_DIS);
		writel(0x0, wrapper_base + WRAPPER_CPU_CLOCK_CONFIG);

		/* Bring ARM9 out of reset */
		writel(0, wrapper_base + WRAPPER_A9SS_SW_RESET);
	}
}

int venus_set_hw_state(struct venus_core *core, bool resume)
{
	int ret;

	if (core->use_tz) {
		ret = qcom_scm_set_remote_state(resume, 0);
		if (resume && ret == -EINVAL)
			ret = 0;
		return ret;
	}

	if (resume) {
		venus_reset_cpu(core);
	} else {
		if (IS_IRIS2(core) || IS_IRIS2_1(core))
			writel(WRAPPER_XTSS_SW_RESET_BIT,
			       core->wrapper_tz_base + WRAPPER_TZ_XTSS_SW_RESET);
		else
			writel(WRAPPER_A9SS_SW_RESET_BIT,
			       core->wrapper_base + WRAPPER_A9SS_SW_RESET);
	}

	return 0;
}

static int venus_load_fw(struct venus_core *core, const char *fwname,
			 phys_addr_t *mem_phys, size_t *mem_size)
{
	const struct firmware *mdt;
	struct resource res;
	struct device *dev;
	ssize_t fw_size;
	void *mem_va;
	int ret;

	*mem_phys = 0;
	*mem_size = 0;

	dev = core->dev;
	venus_fw_checkpoint(core, "reserved-memory lookup start");
	ret = of_reserved_mem_region_to_resource(dev->of_node, 0, &res);
	if (ret) {
		dev_err(dev, "failed to lookup reserved memory-region\n");
		return -EINVAL;
	}
	venus_fw_checkpoint(core, "reserved-memory lookup done");

	venus_fw_checkpoint(core, "request_firmware start");
	ret = request_firmware(&mdt, fwname, dev);
	if (ret < 0)
		return ret;
	venus_fw_checkpoint(core, "request_firmware done");

	venus_fw_checkpoint(core, "MDT size read start");
	fw_size = qcom_mdt_get_size(mdt);
	if (fw_size < 0) {
		ret = fw_size;
		goto err_release_fw;
	}
	venus_fw_checkpoint(core, "MDT size read done");

	*mem_phys = res.start;
	*mem_size = resource_size(&res);

	if (*mem_size < fw_size || fw_size > VENUS_FW_MEM_SIZE) {
		dev_err(dev,
			"firmware %s size %zd does not fit reserved memory %#zx\n",
			fwname, fw_size, *mem_size);
		ret = -EINVAL;
		goto err_release_fw;
	}

	dev_info(dev, "firmware %s: image=%zd reserved=%pa+%#zx mode=%s\n",
		 fwname, fw_size, mem_phys, *mem_size,
		 core->use_tz ? "trusted" : "non-secure");

	venus_fw_checkpoint(core, "reserved-memory memremap start");
	mem_va = memremap(*mem_phys, *mem_size, MEMREMAP_WC);
	if (!mem_va) {
		dev_err(dev, "unable to map memory region %pa size %#zx\n", mem_phys, *mem_size);
		ret = -ENOMEM;
		goto err_release_fw;
	}
	venus_fw_checkpoint(core, "reserved-memory memremap done");

	if (IS_IRIS1(core) && iris1_fw_stage == 1) {
		dev_info(dev,
			 "Iris1 diagnostic map-only stage complete; stopping probe safely\n");
		venus_fw_checkpoint(core, "map-only stop committed");
		memunmap(mem_va);
		ret = -ECANCELED;
		goto err_release_fw;
	}

	venus_fw_checkpoint(core, "qcom_mdt_load start");
	if (core->use_tz)
		ret = qcom_mdt_load(dev, mdt, fwname, VENUS_PAS_ID,
				    mem_va, *mem_phys, *mem_size, NULL);
	else
		ret = qcom_mdt_load_no_init(dev, mdt, fwname, mem_va,
					    *mem_phys, *mem_size, NULL);
	if (!ret)
		venus_fw_checkpoint(core, "qcom_mdt_load done");

	memunmap(mem_va);
err_release_fw:
	release_firmware(mdt);
	return ret;
}

static int venus_boot_no_tz(struct venus_core *core, phys_addr_t mem_phys,
			    size_t mem_size)
{
	struct iommu_domain *iommu;
	struct device *dev;
	int ret;

	dev = core->fw.dev;
	if (!dev)
		return -EPROBE_DEFER;

	iommu = core->fw.iommu_domain;
	core->fw.mapped_mem_size = mem_size;

	ret = iommu_map(iommu, VENUS_FW_START_ADDR, mem_phys, mem_size,
			IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV, GFP_KERNEL);
	if (ret) {
		dev_err(dev, "could not map video firmware region\n");
		return ret;
	}

	venus_reset_cpu(core);

	return 0;
}

static int venus_shutdown_no_tz(struct venus_core *core)
{
	const size_t mapped = core->fw.mapped_mem_size;
	struct iommu_domain *iommu;
	size_t unmapped;
	u32 reg;
	struct device *dev = core->fw.dev;
	void __iomem *wrapper_base = core->wrapper_base;
	void __iomem *wrapper_tz_base = core->wrapper_tz_base;

	if (IS_IRIS2(core) || IS_IRIS2_1(core)) {
		/* Assert the reset to XTSS */
		reg = readl(wrapper_tz_base + WRAPPER_TZ_XTSS_SW_RESET);
		reg |= WRAPPER_XTSS_SW_RESET_BIT;
		writel(reg, wrapper_tz_base + WRAPPER_TZ_XTSS_SW_RESET);
	} else {
		/* Assert the reset to ARM9 */
		reg = readl(wrapper_base + WRAPPER_A9SS_SW_RESET);
		reg |= WRAPPER_A9SS_SW_RESET_BIT;
		writel(reg, wrapper_base + WRAPPER_A9SS_SW_RESET);
	}

	iommu = core->fw.iommu_domain;

	if (core->fw.mapped_mem_size && iommu) {
		unmapped = iommu_unmap(iommu, VENUS_FW_START_ADDR, mapped);

		if (unmapped != mapped)
			dev_err(dev, "failed to unmap firmware\n");
		else
			core->fw.mapped_mem_size = 0;
	}

	return 0;
}

int venus_firmware_cfg(struct venus_core *core)
{
	void __iomem *cpu_cs_base = core->cpu_cs_base;

	if (IS_AR50_LITE(core))
		writel(CPU_CS_VCICMD_ARP_OFF, cpu_cs_base + CPU_CS_VCICMD);

	return 0;
}

int venus_boot(struct venus_core *core)
{
	struct device *dev = core->dev;
	const struct venus_resources *res = core->res;
	const char *fwpath = NULL;
	phys_addr_t mem_phys;
	size_t mem_size;
	u64 auth_start = 0, auth_ns = 0;
	u64 protect_start = 0, protect_ns = 0;
	int shutdown_ret, ret;

	if (IS_IRIS1(core)) {
		if (iris1_fw_stage > 5) {
			dev_err(dev, "invalid Iris1 firmware diagnostic stage %u\n",
				iris1_fw_stage);
			return -EINVAL;
		}

		dev_info(dev,
			 "Iris1 firmware diagnostic configuration: stage=%u checkpoint_ms=%u\n",
			 iris1_fw_stage, min(iris1_fw_checkpoint_ms, 5000U));
		venus_fw_checkpoint(core, "diagnostic configuration committed");
	}

	if (!IS_ENABLED(CONFIG_QCOM_MDT_LOADER) ||
	    (core->use_tz && !qcom_scm_is_available()))
		return -EPROBE_DEFER;

	ret = of_property_read_string_index(dev->of_node, "firmware-name", 0,
					    &fwpath);
	if (ret)
		fwpath = core->res->fwname;

	ret = venus_load_fw(core, fwpath, &mem_phys, &mem_size);
	if (ret) {
		dev_err(dev, "failed to load video firmware %s (%d)\n",
			fwpath, ret);
		return -EINVAL;
	}

	core->fw.mem_size = mem_size;
	core->fw.mem_phys = mem_phys;

	if (IS_IRIS1(core) && iris1_fw_stage == 2) {
		dev_info(dev,
			 "Iris1 diagnostic load-only stage complete; skipping PAS auth-and-reset\n");
		venus_fw_checkpoint(core, "load-only stop committed");
		return -ECANCELED;
	}

	if (core->use_tz) {
		venus_fw_checkpoint(core, "PAS auth-and-reset start");
		auth_start = ktime_get_ns();
		ret = qcom_scm_pas_auth_and_reset(VENUS_PAS_ID);
		auth_ns = ktime_get_ns() - auth_start;
	} else {
		ret = venus_boot_no_tz(core, mem_phys, mem_size);
	}

	if (ret) {
		dev_err(dev, "firmware start failed in %s mode (%d)\n",
			core->use_tz ? "trusted" : "non-secure", ret);
		return ret;
	}

	/*
	 * Once PAS releases Iris1, the firmware can become a bus master.  Do not
	 * put a logging delay between auth/reset and either shutdown or secure
	 * range configuration: an unconfigured firmware was observed to wedge
	 * the NoC within the old 1.5 second diagnostic delay.
	 */
	if (IS_IRIS1(core) && iris1_fw_stage == 3) {
		shutdown_ret = qcom_scm_pas_shutdown(VENUS_PAS_ID);
		dev_info(dev,
			 "Iris1 auth-stop: auth=%llu us shutdown_ret=%d\n",
			 div_u64(auth_ns, NSEC_PER_USEC), shutdown_ret);
		venus_fw_checkpoint(core, "auth-stop committed");
		return shutdown_ret ?: -ECANCELED;
	}

	if (core->use_tz && res->cp_size) {
		/*
		 * Clues for porting using downstream data:
		 * cp_start = 0
		 * cp_size = venus_ns/virtual-addr-pool[0] - yes, address and not size!
		 *   This works, as the non-secure context bank is placed
		 *   contiguously right after the Content Protection region.
		 *
		 * cp_nonpixel_start = venus_sec_non_pixel/virtual-addr-pool[0]
		 * cp_nonpixel_size = venus_sec_non_pixel/virtual-addr-pool[1]
		 */
		protect_start = ktime_get_ns();
		ret = qcom_scm_mem_protect_video_var(res->cp_start,
						     res->cp_size,
						     res->cp_nonpixel_start,
						     res->cp_nonpixel_size);
		protect_ns = ktime_get_ns() - protect_start;
		if (ret) {
			qcom_scm_pas_shutdown(VENUS_PAS_ID);
			dev_err(dev, "set virtual address ranges fail (%d)\n",
				ret);
			return ret;
		}

		if (IS_IRIS1(core) && iris1_fw_stage == 4) {
			shutdown_ret = qcom_scm_pas_shutdown(VENUS_PAS_ID);
			dev_info(dev,
				 "Iris1 protect-stop: auth=%llu us protect=%llu us shutdown_ret=%d\n",
				 div_u64(auth_ns, NSEC_PER_USEC),
				 div_u64(protect_ns, NSEC_PER_USEC), shutdown_ret);
			venus_fw_checkpoint(core, "protect-stop committed");
			return shutdown_ret ?: -ECANCELED;
		}

		if (IS_IRIS1(core) && iris1_fw_stage == 5) {
			unsigned int hold_ms = min(iris1_fw_hold_ms, 5000U);

			dev_info(dev,
				 "Iris1 hold-stop start: auth=%llu us protect=%llu us hold=%u ms\n",
				 div_u64(auth_ns, NSEC_PER_USEC),
				 div_u64(protect_ns, NSEC_PER_USEC), hold_ms);
			msleep(hold_ms);
			shutdown_ret = qcom_scm_pas_shutdown(VENUS_PAS_ID);
			dev_info(dev, "Iris1 hold-stop done: shutdown_ret=%d\n",
				 shutdown_ret);
			venus_fw_checkpoint(core, "hold-stop committed");
			return shutdown_ret ?: -ECANCELED;
		}

		dev_info(dev,
			 "secure video ranges configured: cp=%#x+%#x nonpixel=%#x+%#x (%llu us)\n",
			 res->cp_start, res->cp_size, res->cp_nonpixel_start,
			 res->cp_nonpixel_size,
			 div_u64(protect_ns, NSEC_PER_USEC));
	}

	dev_info(dev, "firmware started in %s mode (auth %llu us)\n",
		 core->use_tz ? "trusted" : "non-secure",
		 div_u64(auth_ns, NSEC_PER_USEC));

	return 0;
}

int venus_shutdown(struct venus_core *core)
{
	int ret;

	if (core->use_tz)
		ret = qcom_scm_pas_shutdown(VENUS_PAS_ID);
	else
		ret = venus_shutdown_no_tz(core);

	return ret;
}

int venus_firmware_check(struct venus_core *core)
{
	const struct firmware_version *req = core->res->min_fw;
	const struct firmware_version *run = &core->venus_ver;

	if (!req)
		return 0;

	if (!is_fw_rev_or_newer(core, req->major, req->minor, req->rev))
		goto error;

	return 0;
error:
	dev_err(core->dev, "Firmware v%d.%d.%d < v%d.%d.%d\n",
		run->major, run->minor, run->rev,
		req->major, req->minor, req->rev);

	return -EINVAL;
}

int venus_firmware_init(struct venus_core *core)
{
	struct platform_device_info info;
	struct iommu_domain *iommu_dom;
	struct platform_device *pdev;
	struct device_node *np;
	int ret;

	np = of_get_child_by_name(core->dev->of_node, "video-firmware");
	if (!np) {
		core->use_tz = true;
		return 0;
	}

	memset(&info, 0, sizeof(info));
	info.fwnode = &np->fwnode;
	info.parent = core->dev;
	info.name = np->name;
	info.dma_mask = DMA_BIT_MASK(32);

	pdev = platform_device_register_full(&info);
	if (IS_ERR(pdev)) {
		of_node_put(np);
		return PTR_ERR(pdev);
	}

	pdev->dev.of_node = np;

	ret = of_dma_configure(&pdev->dev, np, true);
	if (ret) {
		dev_err(core->dev, "dma configure fail\n");
		goto err_unregister;
	}

	core->fw.dev = &pdev->dev;

	iommu_dom = iommu_paging_domain_alloc(core->fw.dev);
	if (IS_ERR(iommu_dom)) {
		dev_err(core->fw.dev, "Failed to allocate iommu domain\n");
		ret = PTR_ERR(iommu_dom);
		goto err_unregister;
	}

	ret = iommu_attach_device(iommu_dom, core->fw.dev);
	if (ret) {
		dev_err(core->fw.dev, "could not attach device\n");
		goto err_iommu_free;
	}

	core->fw.iommu_domain = iommu_dom;

	of_node_put(np);

	return 0;

err_iommu_free:
	iommu_domain_free(iommu_dom);
err_unregister:
	platform_device_unregister(pdev);
	of_node_put(np);
	return ret;
}

void venus_firmware_deinit(struct venus_core *core)
{
	struct iommu_domain *iommu;

	if (!core->fw.dev)
		return;

	iommu = core->fw.iommu_domain;

	iommu_detach_device(iommu, core->fw.dev);

	if (core->fw.iommu_domain) {
		iommu_domain_free(iommu);
		core->fw.iommu_domain = NULL;
	}

	platform_device_unregister(to_platform_device(core->fw.dev));
}
