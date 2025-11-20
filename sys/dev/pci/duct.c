#include <sys/mutex.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <sys/device.h>
#include <sys/vnode.h>
#include <sys/atomic.h>
#include <sys/cdefs.h>
#include <sys/event.h>
#include <sys/selinfo.h>

#include <machine/bus.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcidevs.h>
#include <sys/uio.h>

#include <dev/ductvar.h>

#define TRUE  		1
#define FALSE 		0

#define DUCT_BAR0 	PCI_MAPREG_START /* BAR 0 offset in PCI config space */

/* Versioning */
#define DUCT_R_VMAJ	0x00    /* Major version (RO, 32-bit) */
#define DUCT_R_VMIN	0x04    /* Minor version (RO, 32-bit) */

/* Error flags + hardware address */
#define DUCT_R_FLAGS	0x08    /* Error flags (RO/resettable, 32-bit) */
#define DUCT_R_HWADDR	0x0C    /* Hardware address (RO, 32-bit) */

/* Command ring */
#define DUCT_R_CMDBASE	0x10    /* Command ring base addr (64-bit, WO) */
#define DUCT_R_CMDSHIFT	0x18    /* Command ring size shift (32-bit, WO) */

/* Transmit ring */
#define DUCT_R_TXBASE	0x20    /* TX ring base addr (64-bit, WO) */
#define DUCT_R_TXSHIFT	0x28    /* TX ring size shift (32-bit, WO) */

/* Receive ring */
#define DUCT_R_RXBASE	0x30    /* RX ring base addr (64-bit, WO) */
#define DUCT_R_RXSHIFT	0x38    /* RX ring size shift (32-bit, WO) */

/* Events + doorbell */
#define DUCT_R_EVFLAGS	0x40    /* Event flags (clear-on-read, 32-bit) */
#define DUCT_R_DBELL	0x50    /* Doorbell register (32-bit, WO) */

/* BAR size */
#define DUCT_BAR0_SIZE   0x80

#define DUCT_CMD_START 		0x01
#define DUCT_CMD_STOP 		0x02
#define DUCT_CMD_ADDFILT	0x03
#define DUCT_CMD_RMFILT		0x04
#define DUCT_CMD_FLUSHFILT	0x05

#define DUCT_EV_TXCOMP   (1 << 0)
#define DUCT_EV_RXCOMP   (1 << 1)
#define DUCT_EV_RXDROP   (1 << 2)
#define DUCT_EV_CMDCOMP  (1 << 3)
#define DUCT_EV_RXJUMBO  (1 << 4)

/* 4 4kb pages */
#define DUCT_PKT_MAXLEN (PAGE_SIZE * 4)
#define DUCT_PKT_PAGES 	4
#define DUCT_PKT_LAST_PAGE 3

/*
 * Rings are defined as relative to DUCT_RING_SHIFT since they are all
 * equally sized in this implementation. Verbose definitions are pre-emptive
 * and allow easy adjusting later, but using one global constant helps
 * avoid unnecessary repeated checks for equally sized RX/TX pools
 */
#define DUCT_RING_SHIFT 	6
#define DUCT_RING_SIZE		(1 << DUCT_RING_SHIFT)

#define DUCT_CMDRING_SHIFT	DUCT_RING_SHIFT
#define DUCT_CMDRING_SIZE	(1 << DUCT_CMDRING_SHIFT)
#define DUCT_CMDRING_MASK	(DUCT_CMDRING_SIZE - 1)

#define DUCT_TXRING_SHIFT 	DUCT_RING_SHIFT
#define DUCT_TXRING_SIZE	(1 << DUCT_TXRING_SHIFT)
#define DUCT_TXRING_MASK 	(DUCT_TXRING_SIZE - 1)
#define DUCT_TXBUFF_COUNT	DUCT_PKT_PAGES * DUCT_TXRING_SIZE

#define DUCT_RXRING_SHIFT 	DUCT_RING_SHIFT
#define DUCT_RXRING_SIZE	(1 << DUCT_RXRING_SHIFT)
#define DUCT_RXRING_MASK 	(DUCT_RXRING_SIZE - 1)
#define DUCT_RXBUFF_COUNT	DUCT_PKT_PAGES * DUCT_RXRING_SIZE

#define DUCT_OWNER_HOST		0xAA
#define DUCT_OWNER_DEV 		0x55

#define DUCT_DBELL_HIGH		0x80000000

#define DUCT_RESET_BIT 		0x80000000
#define DUCT_RESET_TIMEOUT 	0xFFFFFFFF
#define DUCT_MASK_ALL		0xFFFFFFFF

#define DUCT_DEV_CMD_SIZE 	32
#define DUCT_DEV_PKT_SIZE	64

#define DUCT_MCAST_BIT		0x80000000u


#define DUCT_PKT_LAST_PTR_OFS (16 + 4*DUCT_PKT_PAGES) + 8*(DUCT_PKT_PAGES - 1)

static int   	duct_match(struct device *, void *, void *);
static void  	duct_attach(struct device *, struct device *, void *);
static int   	duct_filt_read_event(struct knote *, long);
static int   	duct_filt_write_event(struct knote *, long);

struct duct_softc {
	struct device		sc_dev;

	pci_chipset_tag_t	sc_pc;
	pci_intr_handle_t	sc_ih_ev, sc_ih_err;
	void			*sc_ihc_ev, *sc_ihc_err;
	pcitag_t		sc_tag;

	/* Track how many fds are open for pkt processing */
	uint32_t 		sc_open_count;
	uint32_t 		sc_started;

	bus_space_tag_t		sc_memt;
	bus_space_handle_t	sc_memh;
	bus_size_t		sc_mems;

	bus_dma_tag_t		sc_dmat;
	bus_dmamap_t		sc_dmap_i;
	bus_dmamap_t		sc_dmap_o;

	struct mutex		sc_mtx;

	/* CMD Ring */
	struct duct_cmd_desc	*sc_cmdring;
	bus_dma_segment_t	sc_cmdsegs[1];
	int			sc_cmdnsegs;
	bus_size_t		sc_cmdsize;
	unsigned int		sc_cmd_prod;
	unsigned int		sc_cmd_cons;
	bus_dmamap_t		sc_cmdmap;
	bus_addr_t		sc_cmdphys;

	/* TX Ring */
	struct duct_pkt_desc	*sc_txring;
	bus_dma_segment_t	sc_txsegs[1];
	int			sc_txnsegs;
	bus_size_t		sc_txsize;
	unsigned int		sc_tx_prod;
	unsigned int		sc_tx_cons;
	bus_dmamap_t		sc_txmap;
	bus_addr_t		sc_txphys;

	/* TX buffer pool */
	void 			*sc_txbufs[DUCT_TXBUFF_COUNT];
	bus_dmamap_t 		sc_txbuf_maps[DUCT_TXBUFF_COUNT];
	bus_addr_t 		sc_txbuf_phys[DUCT_TXBUFF_COUNT];
	bus_dma_segment_t 	sc_txbuf_segs[DUCT_TXBUFF_COUNT];
	int 			sc_txbuf_nsegs[DUCT_TXBUFF_COUNT];

	/* RX Ring */
	struct duct_pkt_desc	*sc_rxring;
	bus_dma_segment_t	sc_rxsegs[1];
	int			sc_rxnsegs;
	bus_size_t		sc_rxsize;
	unsigned int		sc_rx_next;
	bus_dmamap_t		sc_rxmap;
	bus_addr_t		sc_rxphys;

	/* RX buffer pool */
	void 			*sc_rxbufs[DUCT_RXBUFF_COUNT];
	bus_dmamap_t 		sc_rxbuf_maps[DUCT_RXBUFF_COUNT];
	bus_addr_t 		sc_rxbuf_phys[DUCT_RXBUFF_COUNT];
	bus_dma_segment_t 	sc_rxbuf_segs[DUCT_RXBUFF_COUNT];
	int 			sc_rxbuf_nsegs[DUCT_RXBUFF_COUNT];

	struct klist 		sc_rsel; /* read readiness */
	struct klist 		sc_wsel; /* write readiness */

	int			sc_opening;
	int			sc_closing;
	int			sc_io_inflight;
	int			sc_fatal;
};

struct duct_cmd_desc {
	uint8_t owner;
	uint8_t type;
	uint8_t error;
	uint8_t res1[5];
	uint32_t filtmask;
	uint32_t filtaddr;
	uint64_t res2;
	uint64_t res3;
} __packed;

CTASSERT(sizeof(struct duct_cmd_desc) == DUCT_DEV_CMD_SIZE);
CTASSERT(offsetof(struct duct_cmd_desc, owner) == 0);
CTASSERT(offsetof(struct duct_cmd_desc, type) == 1);
CTASSERT(offsetof(struct duct_cmd_desc, error) == 2);
CTASSERT(offsetof(struct duct_cmd_desc, res1[0]) == 3);
CTASSERT(offsetof(struct duct_cmd_desc, filtmask) == 8);
CTASSERT(offsetof(struct duct_cmd_desc, filtaddr) == 12);
CTASSERT(offsetof(struct duct_cmd_desc, res2) == 16);
CTASSERT(offsetof(struct duct_cmd_desc, res3) == 24);

struct duct_pkt_desc {
	uint8_t owner;
	uint8_t resv[3];
	uint32_t pktlen;			/* used for RX */
	uint32_t length[DUCT_PKT_PAGES];  	/* 4 buffers */
	uint32_t dest;
	uint32_t src;
	uint64_t pointer[DUCT_PKT_PAGES]; 	/* phys address for buffers */
} __packed;

CTASSERT(sizeof(struct duct_pkt_desc) == DUCT_DEV_PKT_SIZE);
CTASSERT(offsetof(struct duct_pkt_desc, owner) == 0);
CTASSERT(offsetof(struct duct_pkt_desc, pktlen) == 4);
CTASSERT(offsetof(struct duct_pkt_desc, length[0]) == 8);
CTASSERT(offsetof(struct duct_pkt_desc, dest) == 8 + 4*DUCT_PKT_PAGES);
CTASSERT(offsetof(struct duct_pkt_desc, src) == 12 + 4*DUCT_PKT_PAGES);
CTASSERT(offsetof(struct duct_pkt_desc, pointer[0]) == 16 + 4*DUCT_PKT_PAGES);
/*
 * acknowledge pointer[3] is not modular since CASSERT calls cannot be
 * multi-line according to cstyle. however, if someone is to change the
 * DUCT_PKT_LAST_PTR_OFS (offset) when changing the size of the loop, this will
 * not compile and thus pointer[x] can be changed. this is the only way.
 */
CTASSERT(offsetof(struct duct_pkt_desc, pointer[3]) == DUCT_PKT_LAST_PTR_OFS);

const struct cfattach duct_ca = {
	.ca_devsize = sizeof(struct duct_softc),
	.ca_match   = duct_match,
	.ca_attach  = duct_attach
};

struct cfdriver duct_cd = {
	.cd_devs    = NULL,
	.cd_name    = "duct",
	.cd_class   = DV_DULL
};

const struct pci_matchid duct_devices[] = {
	{ 0x3301, 0x2000 }
};
static void 	duct_kqdetach_read(struct knote *);
static void 	duct_kqdetach_write(struct knote *);

/* Filterops for read and write */
static const struct filterops duct_read_filtops = {
	.f_flags   = FILTEROP_ISFD,
	.f_attach  = NULL,
	.f_detach  = duct_kqdetach_read,
	.f_event   = duct_filt_read_event,
	.f_modify  = NULL,
	.f_process = NULL,
};

static const struct filterops duct_write_filtops = {
	.f_flags   = FILTEROP_ISFD,
	.f_attach  = NULL,
	.f_detach  = duct_kqdetach_write,
	.f_event   = duct_filt_write_event,
	.f_modify  = NULL,
	.f_process = NULL,
};

static void duct_ring_free(struct duct_softc *, void *, bus_dma_segment_t *,
    int, bus_dmamap_t, bus_addr_t, size_t);
static int 	duct_io_begin(struct duct_softc *);
static void 	duct_io_end(struct duct_softc *);
static void 	duct_free_rx_pool(struct duct_softc *);
static void 	duct_free_tx_pool(struct duct_softc *);
static int 	duct_reset(struct duct_softc *);
static void 	duct_rx_handover(struct duct_softc *);
static int 	duct_cmd_exec(struct duct_softc *, uint32_t, uint32_t, uint32_t,
    uint8_t *);

static void
duct_cmd_desc_init(struct duct_cmd_desc *d)
{
	d->owner    = DUCT_OWNER_HOST; 	/* owner */
	d->type	    = 0;
	d->error    = 0;
	d->filtmask = 0;
	d->filtaddr = 0;
}

static void
duct_pkt_init(struct duct_pkt_desc *d)
{
	d->owner  = DUCT_OWNER_HOST;
	d->pktlen = 0;
	for (int i = 0; i < DUCT_PKT_PAGES; i++) {
		d->length[i]  = 0;
		d->pointer[i] = 0;
	}
	d->dest = 0;
	d->src  = 0;
}


static int
duct_match(struct device *parent, void *match, void *aux)
{
	struct pci_attach_args *pa = aux;
	return (pci_matchbyid(pa, duct_devices, 1));
}

static int
duct_ring_alloc(struct duct_softc *sc, size_t desc_size, size_t count,
    void **ring_out, bus_dma_segment_t *segs, int *nsegs_out,
    bus_dmamap_t *map_out, bus_addr_t *phys_out, size_t *size_out)
{
	size_t size;
	int error, nsegs = 0;

	/* extra defensive */
	if (desc_size == 0 || count == 0)
		return EINVAL;

	size = desc_size * count;

	/* Defensive init so caller never sees garbage on failure */
	*ring_out = NULL;
	*phys_out = 0;
	*size_out = 0;
	*map_out = NULL;
	if (nsegs_out) *nsegs_out = 0;

	error = bus_dmamap_create(sc->sc_dmat, size,
	    1, size, 0,
	    BUS_DMA_WAITOK | BUS_DMA_ALLOCNOW | BUS_DMA_64BIT, map_out);
	if (error != 0)
		return ENOMEM;

	error = bus_dmamem_alloc(sc->sc_dmat, size, PAGE_SIZE, 0,
	    segs,  1, &nsegs, BUS_DMA_WAITOK | BUS_DMA_ZERO);
	if (error != 0) {
		bus_dmamap_destroy(sc->sc_dmat, *map_out);
		*map_out = NULL;
		return ENOMEM;
	}

	error = bus_dmamem_map(sc->sc_dmat, segs, nsegs, size,
	    (caddr_t *)ring_out, BUS_DMA_WAITOK | BUS_DMA_COHERENT);
	if (error != 0) {
		bus_dmamem_free(sc->sc_dmat, segs, nsegs);
		bus_dmamap_destroy(sc->sc_dmat, *map_out);
		*map_out = NULL;
		return ENOMEM;
	}

	error = bus_dmamap_load(sc->sc_dmat, *map_out, *ring_out, size,
	    NULL, BUS_DMA_WAITOK);
	if (error != 0) {
		/* load failed: undo map + alloc + map_create */
		bus_dmamem_unmap(sc->sc_dmat, *ring_out, size);
		*ring_out = NULL;
		bus_dmamem_free(sc->sc_dmat, segs, nsegs);
		bus_dmamap_destroy(sc->sc_dmat, *map_out);
		*map_out = NULL;
		return ENOMEM;
	}

	*phys_out = (*map_out)->dm_segs[0].ds_addr;
	*size_out = size;
	if (nsegs_out) *nsegs_out = nsegs;
	return 0;
}

/*
 * Built as one function for both descriptor types since implementation
 * assumes equal amount of packets - see defines.
 */
static int
duct_bufpool_alloc(struct duct_softc *sc,
    int total,
    bus_dma_segment_t *buffer_segs,
    int *nbuffer_segs,
    void **kva_out,
    bus_dmamap_t *map_out,
    bus_addr_t *phys_out)
{
	int idx = 0;

	MUTEX_ASSERT_UNLOCKED(&sc->sc_mtx);

	/*
	 * really defensive and pre-emptive but makes unwinding easier when
	 * fail
	 */
	for (idx = 0; idx < total; idx++) {
		map_out[idx]	   = NULL;
		kva_out[idx]	   = NULL;
		phys_out[idx]	   = 0;
		nbuffer_segs[idx]  = 0;
		/* buffer_seg left undefined until alloc succeeds */
	}

	for (int i = 0; i < total; i++) {
		for (int j = 0; j < DUCT_PKT_PAGES; j++) {
			idx = i * DUCT_PKT_PAGES + j;

			/* create DMA map object */
			if (bus_dmamap_create(
			    sc->sc_dmat,
			    PAGE_SIZE,
			    1,
			    PAGE_SIZE,
			    0,
			    BUS_DMA_WAITOK,
			    &map_out[idx]) != 0) {
				printf("%s: dmamap_create failed at idx=%d\n",
				    sc->sc_dev.dv_xname, idx);
				goto fail;
			}

			/* allocate physically contiguous dma memory page */
			bus_dma_segment_t seg;
			int nsegs = 0;
			if (bus_dmamem_alloc(
			    sc->sc_dmat,
			    PAGE_SIZE,
			    PAGE_SIZE,
			    0,
			    &seg,
			    1,
			    &nsegs,
			    BUS_DMA_WAITOK | BUS_DMA_ZERO) != 0) {
				printf("%s: dmamem_alloc failed at idx=%d\n",
				    sc->sc_dev.dv_xname, idx);
				goto fail;
			}
			buffer_segs[idx]  = seg;
			nbuffer_segs[idx] = nsegs;

			/* map physical address to kernel virtual address */
			if (bus_dmamem_map(
			    sc->sc_dmat,
			    &buffer_segs[idx],
			    nbuffer_segs[idx],
			    PAGE_SIZE,
			    (caddr_t *)&kva_out[idx],
			    BUS_DMA_WAITOK | BUS_DMA_COHERENT) != 0) {
				printf("%s: dmamem_map failed at idx=%d\n",
				    sc->sc_dev.dv_xname, idx);
				/* undo step 2 before bailing */
				bus_dmamem_free(sc->sc_dmat, &buffer_segs[idx],
				    nbuffer_segs[idx]);
				buffer_segs[idx].ds_addr = 0;
				nbuffer_segs[idx] = 0;
				goto fail;
			}

			/* load the map so device can use DMA on it */
			if (bus_dmamap_load(
			    sc->sc_dmat,
			    map_out[idx],
			    kva_out[idx],
			    PAGE_SIZE,
			    NULL,
			    BUS_DMA_WAITOK) != 0) {
				printf("%s: dmamap_load failed at idx=%d\n",
				    sc->sc_dev.dv_xname, idx);
				/*
				 * undo step 3 + 2; map_out[idx] exists but
				 * not loaded
				 */
				bus_dmamem_unmap(
				    sc->sc_dmat,
				    kva_out[idx],
				    PAGE_SIZE);
				kva_out[idx] = NULL;
				bus_dmamem_free(
				    sc->sc_dmat,
				    &buffer_segs[idx],
				    nbuffer_segs[idx]);
				nbuffer_segs[idx] = 0;
				goto fail;
			}

			/* success for this entry */
			phys_out[idx] = map_out[idx]->dm_segs[0].ds_addr;
		}
	}

	return 0;

fail:
	/* unwind all entries including current one that spiked err. */
	for (int k = 0; k <= idx; k++) {
		if (map_out[k] != NULL) {
			/*
			 * If dmamap_load succeeded, dm_nsegs > 0 unload first.
			 * If load never happened, dm_nsegs should be 0,
			 * no unlock
			 */
			if (map_out[k]->dm_nsegs > 0) {
				bus_dmamap_unload(sc->sc_dmat, map_out[k]);
			}
		}

		if (kva_out[k] != NULL) {
			bus_dmamem_unmap(sc->sc_dmat, kva_out[k], PAGE_SIZE);
			kva_out[k] = NULL;
		}

		if (nbuffer_segs[k] > 0) {
			bus_dmamem_free(sc->sc_dmat, &buffer_segs[k],
			    nbuffer_segs[k]);
			nbuffer_segs[k] = 0;
		}

		if (map_out[k] != NULL) {
			bus_dmamap_destroy(sc->sc_dmat, map_out[k]);
			map_out[k] = NULL;
		}

		phys_out[k] = 0;
	}

	return (ENOMEM);
}

/* Clears the pointers and lenght values for close */
static void
duct_clear_rx_ptrs(struct duct_softc *sc)
{
	for (int i = 0; i < DUCT_RXRING_SIZE; i++) {
		for (int j = 0; j < DUCT_PKT_PAGES; j++) {
			sc->sc_rxring[i].pointer[j] = 0;
			sc->sc_rxring[i].length[j]  = 0;
		}
	}
}

static int
duct_intr_event(void *arg)
{
	struct duct_softc *sc = arg;
	/* Step 1: read EVFLAGS (clear-on-read) */
	uint32_t ev = bus_space_read_4(sc->sc_memt, sc->sc_memh,
	    DUCT_R_EVFLAGS);

	if (ev == 0)
		return 0;   /* not our interrupt */

	/*
	 * NOTE: the mutexes are being entered and exited at every step because
	 * there is a non-zero chance that EVFLAGS has 2 activated flags, not
	 * just one.
	 */

	/* command completion */
	if (ev & DUCT_EV_CMDCOMP) {
		mtx_enter(&sc->sc_mtx);

		while (sc->sc_cmd_cons != sc->sc_cmd_prod) {
			unsigned int idx = sc->sc_cmd_cons & DUCT_CMDRING_MASK;
			struct duct_cmd_desc *desc = &sc->sc_cmdring[idx];

			bus_dmamap_sync(sc->sc_dmat, sc->sc_cmdmap,
			    idx * sizeof(*desc), sizeof(*desc),
			    BUS_DMASYNC_POSTREAD);

			if ((desc->owner) != DUCT_OWNER_HOST)
				break;

			membar_consumer();
			wakeup(desc);
			sc->sc_cmd_cons++;
		}

		mtx_leave(&sc->sc_mtx);
	}

	/* RX packet ready */
	if (ev & DUCT_EV_RXCOMP) {
		/*
		 * clean up rx since its useless after we get callback and
		 * get it ready for next time, logic is a lot of copy paste
		 * from cmd
		 */
		knote(&sc->sc_rsel, 0);
		mtx_enter(&sc->sc_mtx);
		wakeup(&sc->sc_rx_next);
		mtx_leave(&sc->sc_mtx);
	}

	/* TX packet finished */
	if (ev & DUCT_EV_TXCOMP) {
		/*
		 * clean up tx since its useless after we get callback and
		 * get it ready for next time, logic is a lot of copy paste
		 * from cmd
		 */
		mtx_enter(&sc->sc_mtx);
		while (sc->sc_tx_cons != sc->sc_tx_prod) {
			unsigned int idx = sc->sc_tx_cons & DUCT_TXRING_MASK;
			struct duct_pkt_desc *desc = &sc->sc_txring[idx];

			bus_dmamap_sync(sc->sc_dmat, sc->sc_txmap,
			    idx * sizeof(*desc), sizeof(*desc),
			    BUS_DMASYNC_POSTREAD);

			if ((desc->owner) != DUCT_OWNER_HOST)
				break;

			membar_consumer();

			/*
			 * can zero length, no need to zero pages since they
			 * will be overwritten/not read depending on length
			 */
			for (int j = 0; j < DUCT_PKT_PAGES; j++) {
				desc->length[j] = 0;
			}

			wakeup(&sc->sc_tx_cons);
			mtx_leave(&sc->sc_mtx);
			knote(&sc->sc_wsel, 0);
			mtx_enter(&sc->sc_mtx);

			/* Reclaim slot */
			sc->sc_tx_cons++;
		}
		mtx_leave(&sc->sc_mtx);
	}

	if (ev & DUCT_EV_RXDROP) {
		printf("RXDROP on device [%s]: Packet dropped due to lack of "
		    "available buffers! Discarding...\n", sc->sc_dev.dv_xname);
	}

	if (ev & DUCT_EV_RXJUMBO) {
		printf("RXJUMBO on device [%s]: Packet too large! "
		    "Discarding...\n", sc->sc_dev.dv_xname);
	}

	return (1);
}

static int
duct_intr_fatal(void *arg)
{
	struct duct_softc *sc = arg;

	/* read FLAGS from BAR */
	uint32_t flags = bus_space_read_4(sc->sc_memt, sc->sc_memh,
	    DUCT_R_FLAGS);

	printf("%s: FATAL error, FLAGS=0x%08x — device halted\n",
	    sc->sc_dev.dv_xname, flags);

	knote(&sc->sc_rsel, 0);
	knote(&sc->sc_wsel, 0);
	mtx_enter(&sc->sc_mtx);
	sc->sc_fatal   = 1;
	sc->sc_started = 0;
	sc->sc_closing = 1;

	wakeup(&sc->sc_rx_next);
	wakeup(&sc->sc_tx_cons);


	for (int i = 0; i < DUCT_CMDRING_SIZE; i++)
		wakeup(&sc->sc_cmdring[i]);
	for (int i = 0; i < DUCT_TXRING_SIZE; i++)
		wakeup(&sc->sc_txring[i]);
	for (int i = 0; i < DUCT_RXRING_SIZE; i++)
		wakeup(&sc->sc_rxring[i]);

	mtx_leave(&sc->sc_mtx);

	if (duct_reset(sc) != 0) {
		printf("%s: reset failed! Device permanently dead.\n",
		    sc->sc_dev.dv_xname);
	} else {
		printf("%s: device successfully reset!\n",
		    sc->sc_dev.dv_xname);
	}

	return (1);
}

static void
duct_program_bar_cmd(struct duct_softc *sc)
{
	bus_space_write_8(sc->sc_memt, sc->sc_memh,
	    DUCT_R_CMDBASE, sc->sc_cmdphys);
	bus_space_write_4(sc->sc_memt, sc->sc_memh,
	    DUCT_R_CMDSHIFT, DUCT_CMDRING_SHIFT);

}

static void
duct_program_bar_xmit(struct duct_softc *sc)
{
	bus_space_write_8(sc->sc_memt, sc->sc_memh,
	    DUCT_R_TXBASE, sc->sc_txphys);
	bus_space_write_4(sc->sc_memt, sc->sc_memh,
	    DUCT_R_TXSHIFT, DUCT_TXRING_SHIFT);

	bus_space_write_8(sc->sc_memt, sc->sc_memh,
	    DUCT_R_RXBASE, sc->sc_rxphys);
	bus_space_write_4(sc->sc_memt, sc->sc_memh,
	    DUCT_R_RXSHIFT, DUCT_RXRING_SHIFT);
}

static void
duct_attach(struct device *parent, struct device *self, void *aux)
{

	struct duct_softc *sc = (struct duct_softc *)self;
	struct pci_attach_args *pa = aux;

	/* attach arguments */
	sc->sc_pc = pa->pa_pc;
	sc->sc_tag = pa->pa_tag;
	sc->sc_dmat = pa->pa_dmat;

	mtx_init(&sc->sc_mtx, IPL_BIO);
	pcireg_t memtype = pci_mapreg_type(sc->sc_pc, sc->sc_tag, DUCT_BAR0);

	if (PCI_MAPREG_TYPE(memtype) != PCI_MAPREG_TYPE_MEM) {
		printf(": BAR0 is not a memory region\n");
		goto fail;
	}

	/* map bar memory */
	if (pci_mapreg_map(pa, DUCT_BAR0, memtype, 0, &sc->sc_memt,
	    &sc->sc_memh, NULL, &sc->sc_mems, 0)) {

		printf(": unable to map register memory\n");
		goto fail;
	}

	if (sc->sc_mems != DUCT_BAR0_SIZE) {
		printf(": BAR is incorrectly sized! (%zx but expected"
		    " 0x%X) \n", sc->sc_mems, DUCT_BAR0_SIZE);
		goto fail;
	}

	uint32_t maj = bus_space_read_4(sc->sc_memt, sc->sc_memh, DUCT_R_VMAJ);
	uint32_t min = bus_space_read_4(sc->sc_memt, sc->sc_memh, DUCT_R_VMIN);
	uint32_t addr = bus_space_read_4(sc->sc_memt, sc->sc_memh,
	    DUCT_R_HWADDR);

	/*
	 * This is as thorough as attachment to the correct device goes.
	 * The reason is that there is no expected version number given in
	 * the spec or the device spec, so it would be wrong to check the
	 * version. The only certain things are the vendor (0x3301), the
	 * product (0x2000), and the BAR size/memtype checks.
	 */
	printf("%s: duct device v%d.%d hwaddr=0x%08x\n", sc->sc_dev.dv_xname,
	    maj, min, addr);

	/* Cmd ring allocation + initialisation */
	if (duct_ring_alloc(
	    sc,
	    sizeof(struct duct_cmd_desc),
	    DUCT_CMDRING_SIZE,
	    (void**)&sc->sc_cmdring,
	    sc->sc_cmdsegs,
	    &sc->sc_cmdnsegs,
	    &sc->sc_cmdmap,
	    &sc->sc_cmdphys,
	    &sc->sc_cmdsize) != 0) {
		printf(": failed to allocate CMD ring\n");
		goto fail;
	}
	for (int i = 0; i < DUCT_CMDRING_SIZE; i++) {
		duct_cmd_desc_init(&sc->sc_cmdring[i]);
	}

	/* RX Ring allocation + initialisation */
	if (duct_ring_alloc(
	    sc,
	    sizeof(struct duct_pkt_desc),
	    DUCT_RXRING_SIZE,
	    (void**)&sc->sc_rxring,
	    sc->sc_rxsegs,
	    &sc->sc_rxnsegs,
	    &sc->sc_rxmap,
	    &sc->sc_rxphys,
	    &sc->sc_rxsize) != 0) {
		printf(": failed to allocate RX ring\n");
		goto fail;
	}
	for (int i = 0; i < DUCT_RXRING_SIZE; i++) {
		duct_pkt_init(&sc->sc_rxring[i]);
	}

	/* TX Ring allocation + initialisation */
	if (duct_ring_alloc(
	    sc,
	    sizeof(struct duct_pkt_desc),
	    DUCT_TXRING_SIZE,
	    (void**)&sc->sc_txring,
	    sc->sc_txsegs,
	    &sc->sc_txnsegs,
	    &sc->sc_txmap,
	    &sc->sc_txphys,
	    &sc->sc_txsize) != 0) {
		printf(": failed to allocate TX ring\n");
		goto fail;
	}
	for (int i = 0; i < DUCT_TXRING_SIZE; i++) {
		duct_pkt_init(&sc->sc_txring[i]);
	}

	/* initialise indexes for each ring */
	sc->sc_cmd_prod = sc->sc_cmd_cons = 0;
	sc->sc_closing = 0;
	sc->sc_started = 0;
	sc->sc_fatal = 0;

	/* Write ring positions */
	bus_dmamap_sync(sc->sc_dmat, sc->sc_cmdmap, 0, sc->sc_cmdsize,
	    BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(sc->sc_dmat, sc->sc_txmap, 0, sc->sc_txsize,
	    BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(sc->sc_dmat, sc->sc_rxmap, 0, sc->sc_rxsize,
	    BUS_DMASYNC_PREWRITE);

	duct_program_bar_cmd(sc);

	/* interrupt handlers */
	const char *intrstr;

	/* Map MSI-X vector 0 (events) */
	if (pci_intr_map_msix(pa, 0, &sc->sc_ih_ev) != 0) {
		printf("%s: can't map MSI-X vec0\n", sc->sc_dev.dv_xname);
		goto fail;
	}
	intrstr = pci_intr_string(sc->sc_pc, sc->sc_ih_ev);
	sc->sc_ihc_ev = pci_intr_establish(
	    sc->sc_pc,
	    sc->sc_ih_ev,
	    IPL_BIO,
	    duct_intr_event,
	    sc,
	    sc->sc_dev.dv_xname);

	if (sc->sc_ihc_ev == NULL) {
		printf("%s: can't establish MSI-X vec0 at %s\n",
		    sc->sc_dev.dv_xname, intrstr);
		goto fail;

	}
	printf("%s: MSI-X vec0 (events) at %s\n",
	    sc->sc_dev.dv_xname, intrstr);

	/* Map MSI-X vector 1 (fatal) */
	if (pci_intr_map_msix(pa, 1, &sc->sc_ih_err) != 0) {
		printf("%s: can't map MSI-X vec1\n", sc->sc_dev.dv_xname);
		goto fail;
	}

	intrstr = pci_intr_string(sc->sc_pc, sc->sc_ih_err);
	sc->sc_ihc_err = pci_intr_establish(
	    sc->sc_pc,
	    sc->sc_ih_err,
	    IPL_BIO,
	    duct_intr_fatal,
	    sc,
	    sc->sc_dev.dv_xname);

	if (sc->sc_ihc_err == NULL) {
		printf("%s: can't establish MSI-X vec1 at %s\n",
		    sc->sc_dev.dv_xname, intrstr);
		goto fail;
	}
	printf("%s: MSI-X vec1 (fatal) at %s\n", sc->sc_dev.dv_xname, intrstr);

	/* Clear any stale events before issuing first command */
	(void)bus_space_read_4(sc->sc_memt, sc->sc_memh, DUCT_R_EVFLAGS);

	klist_init(&sc->sc_rsel, NULL, NULL);
	klist_init(&sc->sc_wsel, NULL, NULL);

	printf("%s: attach successful!\n", sc->sc_dev.dv_xname);
	printf("\n");
	return;

fail:
	/* Disestablish MSI-X */
	if (sc->sc_ihc_err) {
		pci_intr_disestablish(sc->sc_pc, sc->sc_ihc_err);
		sc->sc_ihc_err = NULL;
	}
	if (sc->sc_ihc_ev) {
		pci_intr_disestablish(sc->sc_pc, sc->sc_ihc_ev);
		sc->sc_ihc_ev = NULL;
	}

	/* Free TX ring if allocated */
	if (sc->sc_txring) {
		duct_ring_free(
		    sc,
		    sc->sc_txring,
		    sc->sc_txsegs,
		    sc->sc_txnsegs,
		    sc->sc_txmap,
		    sc->sc_txphys,
		    sc->sc_txsize);

		sc->sc_txring = NULL;
		sc->sc_txphys = 0;
		sc->sc_txsize = 0;
		sc->sc_txmap = NULL;
		sc->sc_txnsegs = 0;
	}

	/* Free RX ring if allocated */
	if (sc->sc_rxring) {
		duct_ring_free(
		    sc,
		    sc->sc_rxring,
		    sc->sc_rxsegs,
		    sc->sc_rxnsegs,
		    sc->sc_rxmap,
		    sc->sc_rxphys,
		    sc->sc_rxsize);

		sc->sc_rxring = NULL;
		sc->sc_rxphys = 0;
		sc->sc_rxsize = 0;
		sc->sc_rxmap = NULL;
		sc->sc_rxnsegs = 0;
	}

	/* Free CMD ring if allocated */
	if (sc->sc_cmdring) {
		duct_ring_free(
		    sc,
		    sc->sc_cmdring,
		    sc->sc_cmdsegs,
		    sc->sc_cmdnsegs,
		    sc->sc_cmdmap,
		    sc->sc_cmdphys,
		    sc->sc_cmdsize);
		sc->sc_cmdring = NULL;
		sc->sc_cmdphys = 0;
		sc->sc_cmdsize = 0;
		sc->sc_cmdmap = NULL;
		sc->sc_cmdnsegs = 0;
	}

	/* Unmap BAR last */
	if (sc->sc_memh && sc->sc_mems) {
		bus_space_unmap(sc->sc_memt, sc->sc_memh, sc->sc_mems);
		sc->sc_memt = 0;
		sc->sc_memh = 0;
		sc->sc_mems = 0;
	}
}

static struct duct_softc *
duct_lookup(dev_t dev)
{
	dev_t unit = minor(dev);
	struct duct_softc *sc;

	if (unit >= duct_cd.cd_ndevs)
		return (NULL);

	sc = duct_cd.cd_devs[unit];

	return (sc);
}

/*
 * submits the command to the device, returns slot index in *slotp on
 * success.
 */
static int
duct_cmd_submit_locked(struct duct_softc *sc,
    uint32_t cmd_type, uint32_t filtmask, uint32_t filtaddr,
    unsigned int *slotp)
{
	unsigned int prod = sc->sc_cmd_prod & DUCT_CMDRING_MASK;
	struct duct_cmd_desc *desc = &sc->sc_cmdring[prod];

	/* Function assumes locked state */
	MUTEX_ASSERT_LOCKED(&sc->sc_mtx);

	/* Wait for slot to be free */
	while ((desc->owner) != DUCT_OWNER_HOST) {
		int err = msleep_nsec(desc, &sc->sc_mtx, PRIBIO, "ductcmq",
		    INFSLP);
		if (err)
			return (err);
	}

	desc->type	= cmd_type;
	desc->filtmask  = filtmask;
	desc->filtaddr  = filtaddr;
	desc->error	= 0;

	/* Make desc visible to device. */
	bus_dmamap_sync(sc->sc_dmat, sc->sc_cmdmap,
	    prod * sizeof(*desc), sizeof(*desc),
	    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);

	membar_producer();
	desc->owner = DUCT_OWNER_DEV;   /* hand to device */
	membar_producer();

	bus_dmamap_sync(sc->sc_dmat, sc->sc_cmdmap,
	    prod * sizeof(*desc), sizeof(*desc), BUS_DMASYNC_PREWRITE);

	/* Bon voyage */
	bus_space_barrier(sc->sc_memt, sc->sc_memh, 0, DUCT_BAR0_SIZE,
	    BUS_SPACE_BARRIER_WRITE);
	bus_space_write_4(sc->sc_memt, sc->sc_memh, DUCT_R_DBELL, prod);

	sc->sc_cmd_prod++;
	*slotp = prod;
	return (0);
}

/* command execute wrapper that waits for completion */
static int
duct_cmd_exec(struct duct_softc *sc, uint32_t cmd_type,
    uint32_t filtmask, uint32_t filtaddr, uint8_t *out_cmd_err)
{
	int error;
	unsigned int slot;
	struct duct_cmd_desc *desc;

	/* should be strictly unlocked due to heavy lock usage */
	MUTEX_ASSERT_UNLOCKED(&sc->sc_mtx);

	if ((error = duct_io_begin(sc)) != 0)
		return error;

	mtx_enter(&sc->sc_mtx);
	if (sc->sc_closing || sc->sc_fatal) {
		mtx_leave(&sc->sc_mtx);
		duct_io_end(sc);
		return EIO;
	}

	error = duct_cmd_submit_locked(sc, cmd_type, filtmask, filtaddr, &slot);
	if (error) {
		mtx_leave(&sc->sc_mtx);
		duct_io_end(sc);
		return error;
	}

	KASSERT(slot < DUCT_CMDRING_SIZE);
	desc = &sc->sc_cmdring[slot];


	/* interruptible wait for completion */
	for (;;) {
		if ((desc->owner) == DUCT_OWNER_HOST)
			break;
		if (sc->sc_closing || sc->sc_fatal) {
			error = EIO;
			break;
		}

		error = msleep_nsec(desc, &sc->sc_mtx, PRIBIO | PCATCH,
		    "ductcmd", INFSLP);
		if (error) {
			error = (error == ERESTART) ? EINTR : error;
			break;
		}
	}

	if (error == 0) {
		/* device has written back the desc. */
		bus_dmamap_sync(sc->sc_dmat, sc->sc_cmdmap,
		    slot * sizeof(*desc), sizeof(*desc), BUS_DMASYNC_POSTREAD);
		if (out_cmd_err)
			*out_cmd_err = desc->error;
	}

	mtx_leave(&sc->sc_mtx);
	duct_io_end(sc);
	return error;
}

/* internal teardown path, runs even when sc_closing==1, no PCATCH. */
static int
duct_cmd_exec_teardown(struct duct_softc *sc, uint32_t cmd_type,
    uint32_t filtmask, uint32_t filtaddr, uint8_t *out_cmd_err)
{
	int error = 0;
	unsigned int slot;
	struct duct_cmd_desc *desc;

	mtx_enter(&sc->sc_mtx);
	if (sc->sc_fatal) {
		mtx_leave(&sc->sc_mtx);
		return (EIO);
	}

	error = duct_cmd_submit_locked(sc, cmd_type, filtmask, filtaddr,
	    &slot);
	if (error) {
		mtx_leave(&sc->sc_mtx);
		return (error);
	}

	desc = &sc->sc_cmdring[slot];

	while ((desc->owner) != DUCT_OWNER_HOST) {
		int err = msleep_nsec(desc, &sc->sc_mtx, PRIBIO, "ductcmT",
		    INFSLP);
		if (err) {
			error = err;
			break;
		}
		if (sc->sc_fatal) {
			error = EIO;
			break;
		}
	}

	if (error == 0) {
		bus_dmamap_sync(sc->sc_dmat, sc->sc_cmdmap,
		    slot * sizeof(*desc), sizeof(*desc), BUS_DMASYNC_POSTREAD);

	if (out_cmd_err)
		*out_cmd_err = desc->error;
	}

	mtx_leave(&sc->sc_mtx);
	return (error);
}

int
ductopen(dev_t dev, int mode, int flags, struct proc *p)
{
	struct duct_softc *sc = duct_lookup(dev);
	int error = 0;
	uint8_t cmd_err;

	if (sc == NULL) {
		return (ENXIO);
	}
	/*
	 * Open will never happen during an attach and this info isn't useful
	 * outside of the panic scenario, hence no mtx needed
	 */
	KASSERT(sc->sc_cmdring != NULL);
	KASSERT(sc->sc_txring  != NULL);
	KASSERT(sc->sc_rxring  != NULL);

	mtx_enter(&sc->sc_mtx);

	printf("%s: OPEN enter closing=%d opening=%d started=%d "
	    "open_count=%u\n", sc->sc_dev.dv_xname, sc->sc_closing,
	    sc->sc_opening, sc->sc_started, sc->sc_open_count);

	/* device already opened and ready to go */
	if (sc->sc_started && !sc->sc_closing && !sc->sc_fatal) {
		sc->sc_open_count++;
		mtx_leave(&sc->sc_mtx);
		return (0);
	}


	if (sc->sc_fatal) {
		mtx_leave(&sc->sc_mtx);
		return (EIO);
	}


	while (sc->sc_closing) {
		error = msleep_nsec(&sc->sc_opening, &sc->sc_mtx,
		    PRIBIO|PCATCH, "ductclosing", INFSLP);

		if (error) {
			mtx_leave(&sc->sc_mtx);
			return (error == ERESTART) ? EINTR : error;
		}
		if (sc->sc_fatal) {
			mtx_leave(&sc->sc_mtx);
			return (EIO);
		}
		if (sc->sc_started && !sc->sc_closing) {
			sc->sc_open_count++;
			mtx_leave(&sc->sc_mtx);
			return (0);
		}
	}

	while (sc->sc_opening) {
		error = msleep_nsec(&sc->sc_opening, &sc->sc_mtx, PRIBIO|PCATCH,
		    "ductopening", INFSLP);
		if (error) {
			mtx_leave(&sc->sc_mtx);
			return (error == ERESTART) ? EINTR : error;
		}
		if (sc->sc_fatal) {
			mtx_leave(&sc->sc_mtx);
			return (EIO);
		}
		if (sc->sc_started && !sc->sc_closing) {
			sc->sc_open_count++;
			mtx_leave(&sc->sc_mtx);
			return (0);
		}
	}


	if (!sc->sc_started) {
		sc->sc_opening = 1;
		sc->sc_tx_prod = sc->sc_tx_cons = 0;
		sc->sc_rx_next = 0;

		mtx_leave(&sc->sc_mtx);

		/* clear any stale events before issuing first command */
		(void)bus_space_read_4(sc->sc_memt, sc->sc_memh,
		    DUCT_R_EVFLAGS);

		/* Create buffer memory pools for TX/RX and allocates them */
		error = duct_bufpool_alloc(
		    sc,
		    DUCT_TXRING_SIZE,
		    sc->sc_txbuf_segs,
		    sc->sc_txbuf_nsegs,
		    sc->sc_txbufs,
		    sc->sc_txbuf_maps,
		    sc->sc_txbuf_phys);

		if (error != 0)
			goto out;

		error = duct_bufpool_alloc(
		    sc,
		    DUCT_RXRING_SIZE,
		    sc->sc_rxbuf_segs,
		    sc->sc_rxbuf_nsegs,
		    sc->sc_rxbufs,
		    sc->sc_rxbuf_maps,
		    sc->sc_rxbuf_phys);

		if (error != 0)
			goto out_rx;

		bus_dmamap_sync(sc->sc_dmat, sc->sc_txmap, 0, sc->sc_txsize,
		    BUS_DMASYNC_PREWRITE);
		bus_dmamap_sync(sc->sc_dmat, sc->sc_rxmap, 0, sc->sc_rxsize,
		    BUS_DMASYNC_PREWRITE);

		/* allocate RX pointers and give device the descriptors */
		duct_rx_handover(sc);

		duct_program_bar_xmit(sc);

		error = duct_cmd_exec(sc, DUCT_CMD_START, 0, 0, &cmd_err);

		if (error != 0 || cmd_err != 0) {
			printf("%s: failed START command (Error=%d, "
			    "Device Error=%d)!\n", sc->sc_dev.dv_xname,
			    error, cmd_err);
			error = EIO;
			goto out_cmds;
		}

		/* unicast filter to start */
		uint32_t meaddr = bus_space_read_4(sc->sc_memt, sc->sc_memh,
		    DUCT_R_HWADDR);
		error = duct_cmd_exec(sc, DUCT_CMD_ADDFILT, DUCT_MASK_ALL,
		    meaddr, &cmd_err);

		if (error != 0 || cmd_err != 0) {
			printf("%s: failed ADDFILT command (Error=%d, "
			    "Device Error=%d)!\n", sc->sc_dev.dv_xname,
			    error, cmd_err);
			error = EIO;
			goto out_cmds;
		}

		/*
		 * Every helper function, cmd_exec, bufpool_alloc and others
		 * are blocking by design. ductopen does not return to userland
		 * until every part is complete.
		 */

		printf("%s: device started, hwaddr=0x%08x\n",
		    sc->sc_dev.dv_xname, meaddr);

		mtx_enter(&sc->sc_mtx);
		if (sc->sc_fatal) {
			mtx_leave(&sc->sc_mtx);
			error = EIO;
			goto out_cmds;
		}

		sc->sc_started = 1;
		sc->sc_opening = 0;
		sc->sc_open_count = 1;
		wakeup(&sc->sc_opening);
		mtx_leave(&sc->sc_mtx);
		return (0);
	} else {
		sc->sc_open_count++;
		mtx_leave(&sc->sc_mtx);
		return (0);
	}

out_cmds:
	duct_clear_rx_ptrs(sc);
	duct_free_rx_pool(sc);
out_rx:
	duct_free_tx_pool(sc);
out:
	mtx_enter(&sc->sc_mtx);
	sc->sc_opening = 0;
	sc->sc_started = 0;
	wakeup(&sc->sc_opening);
	mtx_leave(&sc->sc_mtx);
	return (error ? error : EIO);
}

int
ductclose(dev_t dev, int flags, int mode, struct proc *p)
{
	struct duct_softc *sc = duct_lookup(dev);
	uint8_t cmd_err = 0;
	int error = 0;

	if (sc == NULL) {
		return (ENXIO);
	}

	mtx_enter(&sc->sc_mtx);

	if (sc->sc_open_count == 0) {
		printf("%s: close with open_count=0\n", sc->sc_dev.dv_xname);
		mtx_leave(&sc->sc_mtx);
		return 0;
	}

	/* final device exiting, gotta shut it all down */
	if (--sc->sc_open_count == 0 && sc->sc_started) {
		sc->sc_closing = 1;
		wakeup(&sc->sc_closing);

		for (int i = 0; i < DUCT_CMDRING_SIZE; i++)
			wakeup(&sc->sc_cmdring[i]);
		for (int i = 0; i < DUCT_TXRING_SIZE; i++)
			wakeup(&sc->sc_txring[i]);
		for (int i = 0; i < DUCT_RXRING_SIZE; i++)
			wakeup(&sc->sc_rxring[i]);


		while (sc->sc_io_inflight > 0)
			msleep_nsec(&sc->sc_io_inflight, &sc->sc_mtx, PRIBIO,
			    "ductclose", INFSLP);

		mtx_leave(&sc->sc_mtx);
		knote(&sc->sc_rsel, 0);
		knote(&sc->sc_wsel, 0);

		error = duct_cmd_exec_teardown(sc, DUCT_CMD_STOP, 0,
		    0, &cmd_err);

		if (error != 0 || cmd_err != 0) {
			printf("%s: failed STOP command (Error=%d, "
			    "Device error=%d)!\n", sc->sc_dev.dv_xname,
			    error, cmd_err);
		}

		error = duct_cmd_exec_teardown(sc, DUCT_CMD_FLUSHFILT, 0,
		    0, &cmd_err);
		if (error != 0 || cmd_err != 0) {
			printf("%s: failed FLUSHFILT command (Error=%d, "
			    "Device error=%d)!\n", sc->sc_dev.dv_xname,
			    error, cmd_err);
		}

		/* closing is done, reset flags for future open */
		mtx_enter(&sc->sc_mtx);
		sc->sc_started = 0;
		sc->sc_closing = 0;
		sc->sc_fatal = 0;
		(void)bus_space_read_4(sc->sc_memt, sc->sc_memh,
		    DUCT_R_EVFLAGS);
		wakeup(&sc->sc_opening);
		mtx_leave(&sc->sc_mtx);

		duct_clear_rx_ptrs(sc);
		duct_free_rx_pool(sc);
		duct_free_tx_pool(sc);

		printf("%s: device stopped\n", sc->sc_dev.dv_xname);
		return (0);
	}

	mtx_leave(&sc->sc_mtx);
	return (0);
}

static int
duct_reset(struct duct_softc *sc)
{
	uint32_t flags;
	/* Device should not hold mutex since it busy-waits */
	MUTEX_ASSERT_UNLOCKED(&sc->sc_mtx);

	bus_space_write_4(sc->sc_memt, sc->sc_memh, DUCT_R_FLAGS,
	    DUCT_RESET_BIT);
	for (int i = 0; i < DUCT_RESET_TIMEOUT; i++) {
		flags = bus_space_read_4(sc->sc_memt, sc->sc_memh,
		    DUCT_R_FLAGS);
		if (flags == 0) {
			return (0); /* reset complete */
		}
	}
	return (ETIMEDOUT);
}


/* implicit declarations for specifically ioctl */
static int duct_is_multicast_only(uint32_t, uint32_t);
static int duct_errno_add(uint8_t);
static int duct_errno_rm(uint8_t);

int
ductioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	struct duct_softc *sc = duct_lookup(dev);
	int error = 0;
	uint8_t device_err = 0;
	int cmd_sent_err = 0;

	if (sc == NULL)
		return (ENXIO);


	switch (cmd) {
	case DUCTIOC_GET_INFO: {
		struct duct_info_arg *info = (struct duct_info_arg *)data;

		info->duct_major  = bus_space_read_4(sc->sc_memt,
		    sc->sc_memh, DUCT_R_VMAJ);
		info->duct_minor  = bus_space_read_4(sc->sc_memt,
		    sc->sc_memh, DUCT_R_VMIN);
		info->duct_hwaddr = bus_space_read_4(sc->sc_memt,
		    sc->sc_memh, DUCT_R_HWADDR);

		return (0);
	}
	case DUCTIOC_ADD_MCAST: {
		if ((error = duct_io_begin(sc)) != 0)
			return (error);

		const struct duct_mcast_arg *ua = (const struct
		    duct_mcast_arg *)data;

		if (!duct_is_multicast_only(ua->duct_mask, ua->duct_addr)) {
			duct_io_end(sc);
			return (EINVAL);
		}

		cmd_sent_err = duct_cmd_exec(sc, DUCT_CMD_ADDFILT,
		    ua->duct_mask, ua->duct_addr, &device_err);

		duct_io_end(sc);
		if (cmd_sent_err)
			return (cmd_sent_err);

		return (duct_errno_add(device_err));

	}
	case DUCTIOC_RM_MCAST: {
		if ((error = duct_io_begin(sc)) != 0)
			return (error);
		const struct duct_mcast_arg *ua = (const struct
		    duct_mcast_arg *)data;

		if (!duct_is_multicast_only(ua->duct_mask, ua->duct_addr)) {
			duct_io_end(sc);
			return EINVAL;
		}

		cmd_sent_err = duct_cmd_exec(sc, DUCT_CMD_RMFILT,
		    ua->duct_mask, ua->duct_addr, &device_err);

		duct_io_end(sc);

		if (cmd_sent_err)
			return (cmd_sent_err);

		return (duct_errno_rm(device_err));
	}

	default:
		return (ENOTTY);

	}
}

int
ductwrite(dev_t dev, struct uio *uio, int ioflags)
{
	struct duct_softc *sc = duct_lookup(dev);
	size_t len, left, chunk;
	unsigned int idx, base;
	int taken_desc = 0;
	struct duct_pkt_desc *d;
	int error = 0;
	struct duct_packet_hdr hdr;

	if (sc == NULL)
		return (ENXIO);

	if ((error = duct_io_begin(sc)) != 0)
		return (error);

	void *scratch[DUCT_PKT_PAGES] = { NULL, NULL, NULL, NULL };
	size_t scratch_len[DUCT_PKT_PAGES] = { 0, 0, 0, 0 };

	/* hard cap: one TX desc carries ≤ 4*PAGE_SIZE = 16KiB */
	len = uio->uio_resid;
	if (len < sizeof(hdr)) {
		error = EINVAL;
		goto out;
	}

	if (len > sizeof(hdr) + DUCT_PKT_MAXLEN) {
		error = EMSGSIZE; /* 16KiB cap */
		goto out;
	}

	error  = uiomove(&hdr, sizeof(hdr), uio);
	if (error != 0)
		goto out;

	if (hdr.dpkt_length != uio->uio_resid) {
		error = EINVAL;
		goto out;
	}

	/* Allocate staging buffers for packet data */
	for (int j = 0; j < DUCT_PKT_PAGES; j++) {
		scratch[j] = malloc(PAGE_SIZE, M_DEVBUF, M_WAITOK);
		if (scratch[j] == NULL) {
			error = ENOMEM;
			goto out_free;
		}
	}

	left = hdr.dpkt_length;
	for (int j = 0; j < DUCT_PKT_PAGES && left > 0; j++) {
		chunk = (left > PAGE_SIZE) ? PAGE_SIZE : left;
		if (chunk == 0)
			break;

		error = uiomove(scratch[j], chunk, uio);
		if (error != 0)
			goto out_free;

		scratch_len[j] = chunk;
		left -= chunk;
	}

	mtx_enter(&sc->sc_mtx);

	KASSERT(sc->sc_txring != NULL);
	KASSERT(sc->sc_txmap  != NULL);
	KASSERT(sc->sc_tx_prod - sc->sc_tx_cons <= DUCT_TXRING_SIZE);

	if (sc->sc_fatal || !sc->sc_started) {
		error = EIO;
		goto unlock;
	}

retry_space:
	if ((sc->sc_tx_prod - sc->sc_tx_cons) == DUCT_TXRING_SIZE) {
		if (ISSET(ioflags, IO_NDELAY)) {
			error = EAGAIN;
			goto unlock;
		}

		if (sc->sc_closing || sc->sc_fatal) {
			error = EIO;
			goto unlock;
		}

		error = msleep_nsec(&sc->sc_tx_cons, &sc->sc_mtx, PRIBIO|PCATCH,
		    "ducttx", INFSLP);

		if (error) {
			/* because we did this before, EINTR >>> ERESTART */
			error = (error == ERESTART) ? EINTR : error;
			goto unlock;
		}
		/* no harm in 1 more check before retrying for defense */
		if (sc->sc_closing || sc->sc_fatal) {
			error = EIO;
			goto unlock;
		}

		goto retry_space;
	}


	idx = sc->sc_tx_prod & DUCT_TXRING_MASK;
	d   = &sc->sc_txring[idx];
	base = idx * DUCT_PKT_PAGES;

	/* must be host owner before we write shit on */
	bus_dmamap_sync(sc->sc_dmat, sc->sc_txmap,
	    idx * sizeof(*d), sizeof(*d), BUS_DMASYNC_POSTREAD);

	if ((d->owner) != DUCT_OWNER_HOST) {
		/* no free slot yet - wait */
		if (ISSET(ioflags, IO_NDELAY)) {
			error = EAGAIN;
			goto unlock;
		}
		if (sc->sc_fatal || sc->sc_closing) {
			error = EIO;
			goto unlock;
		}

		error = msleep_nsec(&sc->sc_tx_cons, &sc->sc_mtx,
		    PRIBIO|PCATCH, "ducttx", INFSLP);
		if (error) {
			error = (error == ERESTART) ? EINTR : error;
			goto unlock;
		}
		goto retry_space;
	}

	/*
	 * may seem redundant since all packets are recycled, however, nobody
	 * is perfect. re-initialising pointer data quickly is robust and safe
	 * and worth it.
	 */
	for (int j = 0; j < DUCT_PKT_PAGES; j++) {
		d->length[j] = 0;
		d->pointer[j] = 0;
	}
	d->dest = hdr.dpkt_destination;
	d->src  = hdr.dpkt_source;

	/* after you take the desc indicate it's taken so when fail we revert */
	sc->sc_tx_prod++;
	taken_desc = 1;

	base    = idx * DUCT_PKT_PAGES;

	for (int j = 0; j < DUCT_PKT_PAGES; j++) {
		void *dma_buffer = sc->sc_txbufs[base + j];
		int copy_len = scratch_len[j];

		KASSERT(copy_len <= PAGE_SIZE);

		if (base + j >= DUCT_TXBUFF_COUNT) {
			error = EIO;
			goto rollback;
		}

		if (dma_buffer == NULL) {
			error = EIO;
			goto rollback;
		}

		memcpy(dma_buffer, scratch[j], copy_len);

		bus_dmamap_sync(sc->sc_dmat, sc->sc_txbuf_maps[base + j],
		    0, copy_len, BUS_DMASYNC_PREWRITE);

		d->length[j]  = (uint32_t)copy_len;
		d->pointer[j] = sc->sc_txbuf_phys[base + j];
	}

	/* final check before shipping for close */
	if (sc->sc_fatal || sc->sc_closing) {
		error = EIO;
		goto rollback;
	}
	/*
	 * now chunks are written, copy paste post stamp bit from command
	 * and ship
	 */
	bus_dmamap_sync(sc->sc_dmat, sc->sc_txmap, idx * sizeof(*d), sizeof(*d),
	    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);

	membar_producer();
	d->owner = DUCT_OWNER_DEV;
	membar_producer();

	bus_dmamap_sync(sc->sc_dmat, sc->sc_txmap, idx * sizeof(*d),
	    sizeof(*d), BUS_DMASYNC_PREWRITE);

	/* ring doorbell and bon voyage */
	bus_space_barrier(sc->sc_memt, sc->sc_memh, 0, DUCT_BAR0_SIZE,
	    BUS_SPACE_BARRIER_WRITE);
	bus_space_write_4(sc->sc_memt, sc->sc_memh, DUCT_R_DBELL,
	    (idx | DUCT_DBELL_HIGH));

	if (!ISSET(ioflags, IO_NDELAY)) {
		for (;;) {
			bus_dmamap_sync(sc->sc_dmat, sc->sc_txmap,
			    idx * sizeof(*d), sizeof(d->owner),
			    BUS_DMASYNC_POSTREAD);
			if ((d->owner) == DUCT_OWNER_HOST) {
				membar_consumer();
				break;
			}
			if (sc->sc_closing || sc->sc_fatal) {
				error = EIO;
				goto unlock;
			}
			error = msleep_nsec(&sc->sc_tx_cons, &sc->sc_mtx,
			    PRIBIO|PCATCH, "ducttxw", INFSLP);
			if (error) {
				error = (error == ERESTART) ? EINTR : error;
				goto unlock;
			}
		}
	}

	goto unlock;
rollback:
	if (taken_desc) {
		sc->sc_tx_prod--;
		d->dest = d->src = 0;
		for (int i = 0; i < DUCT_PKT_PAGES; i++) {
			d->length[i] = 0;
			d->pointer[i] = 0;
		}
		taken_desc = 0;

		wakeup(&sc->sc_tx_cons);
	}




unlock:

	mtx_leave(&sc->sc_mtx);
	if (!taken_desc && error != 0) {
		knote(&sc->sc_wsel, 0);
	}
out_free:
	for (int j = 0; j < DUCT_PKT_PAGES; j++) {
		if (scratch[j] != NULL) {
			free(scratch[j], M_DEVBUF, PAGE_SIZE);
			scratch[j] = NULL;
		}
	}

out:
	duct_io_end(sc);
	return (error);
}

int
ductread(dev_t dev, struct uio *uio, int ioflags)
{

	/* error checking stuff and blocking ops */
	struct duct_softc *sc = duct_lookup(dev);
	volatile struct duct_pkt_desc *desc;
	unsigned int idx, base;
	size_t pktlen, chunk;
	int error = 0;
	struct duct_packet_hdr hdr;

	void *scratch[DUCT_PKT_PAGES] = { NULL, NULL, NULL, NULL };
	size_t scratch_len[DUCT_PKT_PAGES] = { 0, 0, 0, 0 };

	if (sc == NULL)
		return (ENXIO);

	if ((error = duct_io_begin(sc)) != 0)
		return (error);

	for (int j = 0; j < DUCT_PKT_PAGES; j++) {
		scratch[j] = malloc(PAGE_SIZE, M_DEVBUF, M_WAITOK);
		if (scratch[j] == NULL) {
			error = ENOMEM;
			goto out_free;
		}
	}

	mtx_enter(&sc->sc_mtx);

	KASSERT(sc->sc_rxring != NULL);
	KASSERT(sc->sc_rxmap  != NULL);

	/* truthfully + shamelessly copied logic from tx */
	/* grab ring position we write to */

	/*
	 * idea is that if there are no packets available, keep looping
	 * until we get one - block-waiting
	 */
	for (;;) {
		idx = sc->sc_rx_next & DUCT_RXRING_MASK;
		desc = &sc->sc_rxring[idx];

		bus_dmamap_sync(sc->sc_dmat, sc->sc_rxmap, idx * sizeof(*desc),
		    sizeof(*desc), BUS_DMASYNC_POSTREAD);

		if ((desc->owner) == DUCT_OWNER_HOST)
			break;

		if (ISSET(ioflags, IO_NDELAY)) {
			error = EAGAIN;
			goto unlock_exit;
		}

		if (sc->sc_closing || sc->sc_fatal) {
			error = EIO;
			goto unlock_exit;
		}

		error = msleep_nsec(&sc->sc_rx_next, &sc->sc_mtx,
		    PRIBIO|PCATCH, "ductrx", INFSLP);

		if (error) {
			error = (error == ERESTART) ? EINTR : error;
			goto unlock_exit;
		}
	}
	membar_consumer();

	pktlen = desc->pktlen;

	if (pktlen == 0 || pktlen > DUCT_PKT_MAXLEN) {
		goto recycle_and_unlock;
	}

	/* Either incorrect data amount or incorrect metadata - bin */
	if (uio->uio_resid < sizeof(hdr)) {
		goto recycle_and_unlock;
	}

	/* claim slot for rx packet during mtx to avoid race - defence 100! */
	base = idx * DUCT_PKT_PAGES;
	sc->sc_rx_next++;		/* claim */

	if (base + (DUCT_PKT_PAGES - 1) >= DUCT_RXBUFF_COUNT) {
		error = EIO;
		goto recycle_and_unlock;
	}

	for (int j = 0; j < DUCT_PKT_PAGES; j++) {
		if (sc->sc_rxbufs[base + j] == NULL ||
		    sc->sc_rxbuf_maps[base + j] == NULL) {
			error = EIO;
			goto recycle_and_unlock;
		}
	}

	hdr.dpkt_source = desc->src;
	hdr.dpkt_destination = desc->dest;
	hdr.dpkt_length = pktlen;
	hdr.dpkt_reserved = 0;

	/* reading chunk by chunk from uio */
	size_t remaining = pktlen;
	for (int j = 0; j < DUCT_PKT_PAGES && remaining > 0; j++) {
		void *kva_page = sc->sc_rxbufs[base + j];
		chunk = MIN(remaining, PAGE_SIZE);

		if (chunk == 0)
			break;

		bus_dmamap_sync(sc->sc_dmat, sc->sc_rxbuf_maps[base + j],
		    0, chunk, BUS_DMASYNC_POSTREAD);

		memcpy(scratch[j], kva_page, chunk);
		scratch_len[j] = chunk;

		remaining -= chunk;
	}

recycle_and_unlock:
	for (int j = 0; j < DUCT_PKT_PAGES; j++) {
		desc->length[j] = PAGE_SIZE;
		desc->pointer[j] = sc->sc_rxbuf_phys[base + j];
	}

	desc->pktlen 	= 0;
	desc->dest 	= 0;
	desc->src 	= 0;

	bus_dmamap_sync(sc->sc_dmat, sc->sc_rxmap, idx * sizeof(*desc),
	    sizeof(*desc), BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);

	membar_producer();
	desc->owner = DUCT_OWNER_DEV;
	membar_producer();

	bus_dmamap_sync(sc->sc_dmat, sc->sc_rxmap,
	    idx * sizeof(*desc), sizeof(desc->owner), BUS_DMASYNC_PREWRITE);

unlock_exit:
	mtx_leave(&sc->sc_mtx);

	if (error == 0 && pktlen > 0) {
		error = uiomove(&hdr, sizeof(hdr), uio);
		if (error)
			goto out_free;
	}
	for (int i = 0; i < DUCT_PKT_PAGES && scratch_len[i] > 0; i++) {
		int to_copy = MIN((int)scratch_len[i], (int)uio->uio_resid);
		if (to_copy == 0)
			break;

		error = uiomove(scratch[i], to_copy, uio);
		if (error)
			break;
	}

out_free:
	for (int j = 0; j < DUCT_PKT_PAGES; j++) {
		if (scratch[j] != NULL) {
			free(scratch[j], M_DEVBUF, PAGE_SIZE);
			scratch[j] = NULL;
		}
	}

	duct_io_end(sc);
	return (error);
}

int
ductkqfilter(dev_t dev, struct knote *kn)
{
	struct duct_softc *sc = duct_lookup(dev);
	if (sc == NULL)
		return (ENXIO);

	switch (kn->kn_filter) {
	case EVFILT_READ:
		kn->kn_fop = &duct_read_filtops;
		kn->kn_hook = sc;
		mtx_enter(&sc->sc_mtx);
		klist_insert(&sc->sc_rsel, kn);
		mtx_leave(&sc->sc_mtx);
		break;

	case EVFILT_WRITE:
		kn->kn_fop = &duct_write_filtops;
		kn->kn_hook = sc;
		mtx_enter(&sc->sc_mtx);
		klist_insert(&sc->sc_wsel, kn);
		mtx_leave(&sc->sc_mtx);
		break;

	default:
		return (EINVAL);
	}

	return (0);
}

static void
duct_free_rx_pool(struct duct_softc *sc)
{
	int total = DUCT_RXRING_SIZE * DUCT_PKT_PAGES;

	for (int i = 0; i < total; i++) {
		if (sc->sc_rxbuf_maps[i] == NULL)
			continue;

		bus_dmamap_unload(sc->sc_dmat, sc->sc_rxbuf_maps[i]);

		if (sc->sc_rxbufs[i] != NULL) {
			bus_dmamem_unmap(sc->sc_dmat, sc->sc_rxbufs[i],
			    PAGE_SIZE);
			sc->sc_rxbufs[i] = NULL;
		}

		bus_dmamem_free(sc->sc_dmat,
		    &sc->sc_rxbuf_segs[i],
		    sc->sc_rxbuf_nsegs[i]);

		bus_dmamap_destroy(sc->sc_dmat, sc->sc_rxbuf_maps[i]);
		sc->sc_rxbuf_maps[i] = NULL;

		sc->sc_rxbuf_phys[i] = 0;
	}
}

static void
duct_free_tx_pool(struct duct_softc *sc)
{
	int total = DUCT_TXRING_SIZE * DUCT_PKT_PAGES;

	for (int i = 0; i < total; i++) {
		if (sc->sc_txbuf_maps[i] == NULL)
			continue;

		bus_dmamap_unload(sc->sc_dmat, sc->sc_txbuf_maps[i]);

		if (sc->sc_txbufs[i] != NULL) {
			bus_dmamem_unmap(sc->sc_dmat, sc->sc_txbufs[i],
			    PAGE_SIZE);
			sc->sc_txbufs[i] = NULL;
		}

		bus_dmamem_free(sc->sc_dmat, &sc->sc_txbuf_segs[i],
		    sc->sc_txbuf_nsegs[i]);

		bus_dmamap_destroy(sc->sc_dmat, sc->sc_txbuf_maps[i]);
		sc->sc_txbuf_maps[i] = NULL;

		sc->sc_txbuf_phys[i] = 0;
	}
}

static int
duct_rx_ready(struct duct_softc *sc)
{
	/* function should be called while locked */
	MUTEX_ASSERT_LOCKED(&sc->sc_mtx);
	KASSERT(sc->sc_rxring != NULL);
	unsigned int idx = sc->sc_rx_next & DUCT_RXRING_MASK;
	struct duct_pkt_desc *d = &sc->sc_rxring[idx];

	bus_dmamap_sync(sc->sc_dmat, sc->sc_rxmap,
	    idx * sizeof(*d), sizeof(*d), BUS_DMASYNC_POSTREAD);
	membar_consumer();

	return (d->owner == DUCT_OWNER_HOST);
}

static size_t
duct_rx_next_pktlen(struct duct_softc *sc)
{
	/* function should be locked while called */
	MUTEX_ASSERT_LOCKED(&sc->sc_mtx);
	unsigned int idx = sc->sc_rx_next & DUCT_RXRING_MASK;
	struct duct_pkt_desc *d = &sc->sc_rxring[idx];
	return (d->pktlen);
}

static size_t
duct_tx_free_descs(struct duct_softc *sc)
{
	KASSERT(sc->sc_tx_prod - sc->sc_tx_cons <= DUCT_TXRING_SIZE);
	return (DUCT_TXRING_SIZE - (sc->sc_tx_prod - sc->sc_tx_cons));
}

static int
duct_filt_read_event(struct knote *kn, long hint)
{
	struct duct_softc *sc = kn->kn_hook;
	int ready = 0;

	mtx_enter(&sc->sc_mtx);

	if (duct_rx_ready(sc)) {
		kn->kn_data = duct_rx_next_pktlen(sc);
		ready = 1;
	} else {
		kn->kn_data = 0;
	}
	mtx_leave(&sc->sc_mtx);

	return ready;
}

static int
duct_filt_write_event(struct knote *kn, long hint)
{
	struct duct_softc *sc = kn->kn_hook;
	int ready = 0;

	mtx_enter(&sc->sc_mtx);

	if (duct_tx_free_descs(sc) > 0) {
		kn->kn_data = duct_tx_free_descs(sc);
		ready = 1;
	} else {
		kn->kn_data = 0;
	}
	mtx_leave(&sc->sc_mtx);
	return (ready);
}

static void
duct_kqdetach_read(struct knote *kn)
{
	/* made very clear throughout duct.c that kq is called without mtx */
	struct duct_softc *sc = kn->kn_hook;
	MUTEX_ASSERT_UNLOCKED(&sc->sc_mtx);
	mtx_enter(&sc->sc_mtx);
	klist_remove(&sc->sc_rsel, kn);
	mtx_leave(&sc->sc_mtx);
}

static void
duct_kqdetach_write(struct knote *kn)
{
	/* made very clear throughout duct.c that kq is called without mtx */
	struct duct_softc *sc = kn->kn_hook;
	MUTEX_ASSERT_UNLOCKED(&sc->sc_mtx);
	mtx_enter(&sc->sc_mtx);
	klist_remove(&sc->sc_wsel, kn);
	mtx_leave(&sc->sc_mtx);
}

/* Check if addr is a multicast one - top bit check */
static int
duct_is_multicast_only(uint32_t mask, uint32_t addr)
{
	return ((addr & DUCT_MCAST_BIT) && (mask & DUCT_MCAST_BIT));
}

/* Converts device errno to ioctl expected */
static int
duct_errno_add(uint8_t dev_err)
{
	if (dev_err == 0)
		return (0);
	if (dev_err == 1)
		return (ENOSPC);	/* no more filter space available */
	return (EIO);			/* any other device error */
}

/* Converts device errno to ioctl expected one */
static int
duct_errno_rm(uint8_t dev_err)
{
	if (dev_err == 0)
		return (0);
	if (dev_err == 1)
		return (ENOENT);   /* “No exactly matching filter” */
	return (EIO);
}

static int
duct_io_begin(struct duct_softc *sc)
{
	mtx_enter(&sc->sc_mtx);
	if (sc->sc_closing) {
		mtx_leave(&sc->sc_mtx);
		return (EIO);
	}
	sc->sc_io_inflight++;
	mtx_leave(&sc->sc_mtx);
	return (0);
}

static void
duct_io_end(struct duct_softc *sc)
{
	mtx_enter(&sc->sc_mtx);
	sc->sc_io_inflight--;
	if (sc->sc_closing && sc->sc_io_inflight == 0)
		wakeup(&sc->sc_io_inflight);
	mtx_leave(&sc->sc_mtx);
}

static void
duct_rx_handover(struct duct_softc *sc)
{
	/* slow operation, unlock for pointer allocation */
	KASSERT(sc->sc_rxring != NULL);
	KASSERT(sc->sc_rxbuf_phys != NULL);
	MUTEX_ASSERT_UNLOCKED(&sc->sc_mtx);
	for (int i = 0; i < DUCT_RXRING_SIZE; i++) {
		for (int j = 0; j < DUCT_PKT_PAGES; j++) {
			int idx = i*DUCT_PKT_PAGES + j;
			sc->sc_rxring[i].pointer[j] = sc->sc_rxbuf_phys[idx];
			sc->sc_rxring[i].length[j]  = PAGE_SIZE;
		}
	}
	mtx_enter(&sc->sc_mtx);
	for (int i = 0; i < DUCT_RXRING_SIZE; i++) {
		for (int j = 0; j < DUCT_PKT_PAGES; j++) {
			membar_producer();
			sc->sc_rxring[i].owner = DUCT_OWNER_DEV;
			membar_producer();
		}
	}
	mtx_leave(&sc->sc_mtx);
}

static void
duct_ring_free(struct duct_softc *sc, void *ring, bus_dma_segment_t *segs,
    int nsegs, bus_dmamap_t map, bus_addr_t phys, size_t size)
{
	/* Unload/destroy the map first so the bus stops touching memory */
	if (map) {
		bus_dmamap_unload(sc->sc_dmat, map);
		bus_dmamap_destroy(sc->sc_dmat, map);
	}

	/* Unmap the virtual address */
	if (ring) {
		bus_dmamem_unmap(sc->sc_dmat, (caddr_t)ring, size);
	}

	/* Free the DMA memory backing the ring */
	if (segs && nsegs) {
		bus_dmamem_free(sc->sc_dmat, segs, nsegs);
	}

	/* phys is informational only; nothing to do */
	(void)phys;
}
