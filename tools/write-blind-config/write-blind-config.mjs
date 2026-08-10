// Standalone matter.js controller script to write manufacturer-specific config attributes
// on the roller blind controller: the per-blind BlindConfig cluster (blindLowerTimeMs /
// blindRiseTimeMs / blindRadioChannel / blindRemoteId), and the root-endpoint SystemConfig
// cluster's NumBlinds attribute, which controls how many blind endpoints the device creates.
//
// Why this exists instead of a simpler option:
//  - matter-server refuses to write attributes on clusters it has no built-in definition
//    for (confirmed error: "Attribute 0 on cluster 4294048768 unknown").
//  - @matter/nodejs-shell's `attributes by-id` command only supports reads, not writes
//    (confirmed via its --help output).
// This script commissions itself as an independent Matter controller/fabric admin on
// the device (Matter supports multiple simultaneous admins, so this does NOT remove or
// disturb the existing Home Assistant pairing), then defines the custom clusters' schemas
// itself so it can issue real writes via matter.js's low-level InteractionClient.
//
// The commissioning/connection code here mirrors @matter/nodejs-shell's own
// MatterNode.ts and cmd_commission.ts (read directly from the installed package's
// source to make sure this uses real, working API calls, not guesses). The attribute
// write uses InteractionClient.setAttribute() from @project-chip/matter.js, which needs
// a ClusterType.Attribute descriptor {id, name, schema} — built by hand here via
// @matter/model's AttributeModel, since our clusters aren't ones matter.js knows about.
//
// Setup:
//   npm install                                             (from this directory)
//   node write-blind-config.mjs [--endpoint <n>] --lower <ms> --rise <ms> --channel <n> --remote-id <hex>
//   node write-blind-config.mjs --num-blinds <n>
//   (any subset of --lower/--rise/--channel/--remote-id/--num-blinds is fine, as long as at
//   least one is given; --endpoint selects which blind's BlindConfig cluster the first four
//   target, default 1, and is ignored by --num-blinds, which always targets endpoint 0)
//
// --num-blinds triggers a reboot on the device ~2s after the write completes, to rebuild its
// set of blind endpoints. If combined with per-blind flags in the same invocation, those are
// written first, so they aren't cut off by the reboot.
//
// First run: commissions this script onto the device, which needs a pairing code passed
// via --pairing-code. Get one from Home Assistant: open the blind controller's device
// page, "..." menu -> "Share device". This is only needed once — after that, this
// script's local storage (in this environment's default Matter storage location, keyed
// by the "blind-config-writer" id used below) remembers the pairing and just reconnects,
// so --pairing-code can be omitted on later runs.

// Side-effect-only import: registers the Node.js platform drivers (storage, crypto,
// network, time) with the Environment. Nothing below works without this.
import "@matter/nodejs";

import { Environment } from "@matter/general";
import { AttributeModel } from "@matter/model";
import { AttributeId, ClusterId, EndpointNumber, ManualPairingCodeCodec } from "@matter/types";
import { GeneralCommissioning } from "@matter/types/clusters";
import { CommissioningController } from "@project-chip/matter.js";
import { NodeStateInformation } from "@project-chip/matter.js/device";

const BLIND_CONFIG_CLUSTER_ID = 0xfff1fc00;
const SYSTEM_CONFIG_CLUSTER_ID = 0xfff1fc01;

// Mirrors BlindConfig::kMinChannel/kMaxChannel and BlindConfig::kMaxRemoteId in app_main.cpp.
const MIN_CHANNEL = 0;
const MAX_CHANNEL = 15;
const MAX_REMOTE_ID = 0xffffff;

// Mirrors SystemConfig::kMinBlinds/kMaxBlinds in app_main.cpp.
const MIN_BLINDS = 1;
const MAX_BLINDS = 8;

const DEFAULT_ENDPOINT_ID = 1;

const USAGE =
    "Usage: node write-blind-config.mjs [--endpoint <n>] [--lower <ms>] [--rise <ms>]\n" +
    "                                    [--channel <n>] [--remote-id <hex-or-decimal>]\n" +
    "                                    [--num-blinds <n>] [--pairing-code <code>]\n" +
    "  At least one of --lower/--rise/--channel/--remote-id/--num-blinds is required.\n" +
    `  --endpoint selects which blind's config cluster --lower/--rise/--channel/--remote-id\n` +
    `  target (default ${DEFAULT_ENDPOINT_ID}, the device's serial log shows each blind's endpoint id as\n` +
    `  "Window covering N created with endpoint_id X"). --num-blinds always targets endpoint 0\n` +
    "  regardless of --endpoint, and makes the device reboot ~2s after the write to rebuild its\n" +
    "  set of blind endpoints.\n" +
    "  --pairing-code is only required the first time, before this device is commissioned\n" +
    "  onto this script's fabric (or again after that pairing is lost, e.g. a factory reset).";

// Parses --endpoint/--lower/--rise/--channel/--remote-id/--num-blinds/--pairing-code from argv.
// The value flags are all optional, but at least one must be given — this is a manual write
// utility, so silently doing nothing would be more confusing than refusing to run.
// --pairing-code is optional too: it's only actually needed the first time (or after the device
// stops trusting our fabric); commission() below is what enforces that, since only it knows
// whether commissioning is actually happening.
function parseArgs(argv) {
    const result = { endpointId: DEFAULT_ENDPOINT_ID };
    for (let i = 0; i < argv.length; i++) {
        const arg = argv[i];
        if (arg === "--endpoint") {
            const raw = argv[++i];
            const value = Number(raw);
            if (raw === undefined || !Number.isInteger(value) || value < 1) {
                throw new Error(`${arg} needs a positive integer endpoint id, got ${JSON.stringify(raw)}`);
            }
            result.endpointId = value;
        } else if (arg === "--lower" || arg === "--rise") {
            const key = arg === "--lower" ? "lowerTimeMs" : "riseTimeMs";
            const raw = argv[++i];
            const value = Number(raw);
            if (raw === undefined || !Number.isFinite(value) || value <= 0) {
                throw new Error(`${arg} needs a positive number of milliseconds, got ${JSON.stringify(raw)}`);
            }
            result[key] = value;
        } else if (arg === "--channel") {
            const raw = argv[++i];
            const value = Number(raw);
            if (raw === undefined || !Number.isInteger(value) || value < MIN_CHANNEL || value > MAX_CHANNEL) {
                throw new Error(`${arg} needs an integer between ${MIN_CHANNEL} and ${MAX_CHANNEL}, got ${JSON.stringify(raw)}`);
            }
            result.channel = value;
        } else if (arg === "--remote-id") {
            const raw = argv[++i];
            const value = Number(raw); // Number() understands "0x..." hex strings same as decimal.
            if (raw === undefined || !Number.isInteger(value) || value < 0 || value > MAX_REMOTE_ID) {
                throw new Error(`${arg} needs an integer between 0 and 0x${MAX_REMOTE_ID.toString(16)}, got ${JSON.stringify(raw)}`);
            }
            result.remoteId = value;
        } else if (arg === "--num-blinds") {
            const raw = argv[++i];
            const value = Number(raw);
            if (raw === undefined || !Number.isInteger(value) || value < MIN_BLINDS || value > MAX_BLINDS) {
                throw new Error(`${arg} needs an integer between ${MIN_BLINDS} and ${MAX_BLINDS}, got ${JSON.stringify(raw)}`);
            }
            result.numBlinds = value;
        } else if (arg === "--pairing-code") {
            const raw = argv[++i];
            if (!raw) {
                throw new Error(`${arg} needs a value`);
            }
            result.pairingCode = raw;
        } else {
            throw new Error(`Unrecognized argument: ${arg}\n${USAGE}`);
        }
    }
    if (
        result.lowerTimeMs === undefined &&
        result.riseTimeMs === undefined &&
        result.channel === undefined &&
        result.remoteId === undefined &&
        result.numBlinds === undefined
    ) {
        throw new Error(USAGE);
    }
    return result;
}

// `type` must match the esp_matter_uintN() type the device declared the attribute with
// (see app_main.cpp) — the TLV encoding differs per width, so a mismatch gets rejected.
function buildAttribute(id, name, type = "uint32") {
    return {
        id: AttributeId(id),
        name,
        schema: new AttributeModel({ id, name, type, access: "RW VO" }),
    };
}

const LowerTimeMs = buildAttribute(0x0000, "LowerTimeMs");
const RiseTimeMs = buildAttribute(0x0001, "RiseTimeMs");
const Channel = buildAttribute(0x0002, "Channel", "uint8");
const RemoteId = buildAttribute(0x0003, "RemoteId");
const NumBlinds = buildAttribute(0x0000, "NumBlinds", "uint8");

// Commissions a fresh node using pairingCode and waits for it to be ready. pairingCode is
// only required here, at the point commissioning is actually about to happen — an
// already-commissioned device doesn't need one, so requiring it up front (e.g. in
// parseArgs) would needlessly force it on every run.
async function commission(commissioningController, pairingCode) {
    if (!pairingCode) {
        throw new Error(
            "This device isn't commissioned onto this script's fabric yet (or lost that pairing), so " +
                "--pairing-code is required. Get one from Home Assistant: device page -> \"...\" menu -> " +
                '"Share device".',
        );
    }
    const { shortDiscriminator, passcode } = ManualPairingCodeCodec.decode(pairingCode);
    const nodeId = await commissioningController.commissionNode({
        discovery: {
            identifierData: { shortDiscriminator },
            discoveryCapabilities: { ble: false, onIpNetwork: true },
        },
        passcode,
        commissioning: {
            regulatoryLocation: GeneralCommissioning.RegulatoryLocationType.Outdoor,
            regulatoryCountryCode: "XX",
            onAttestationFailure: () => true,
        },
    });
    console.log(`Commissioned as node ${nodeId}`);

    const node = await commissioningController.connectNode(nodeId);
    if (!node.initialized) {
        await node.events.initialized;
    }
    return node;
}

// Reconnects to a previously-commissioned node, or re-commissions from scratch if the
// device no longer trusts our fabric (e.g. the device was factory-reset, or its fabric
// table was otherwise cleared, since we last ran). NOTE: after that kind of reset, HA's
// own pairing to the device is gone too, so --pairing-code must be the device's original/
// default manual pairing code (whatever was used to first commission it), not an HA
// "Share device" code — HA no longer has any relationship with the device to share.
// connectNode() resolves quickly with a PairedNode built from locally cached data — that's
// what makes node.initialized true, NOT a live CASE handshake. The real connection attempt
// happens afterward in the background, and on failure (e.g. NoSharedTrustRoots) matter.js
// just logs a warning and schedules a retry in 5 minutes rather than rejecting any promise.
// So staleness has to be detected by explicitly waiting for a Connected state signal
// ourselves, with our own timeout — waiting on node.initialized alone will not surface it.
async function connectAndVerify(commissioningController, nodeId, timeoutMs = 15000) {
    let resolveConnected, rejectConnected;
    const connected = new Promise((resolve, reject) => {
        resolveConnected = resolve;
        rejectConnected = reject;
    });
    // If connectNode() below rejects before anything awaits `connected`, the timeout would
    // otherwise fire later and reject an already-abandoned promise, printing a spurious
    // "unhandled rejection" warning. This no-op handler keeps that harmless but silent.
    connected.catch(() => {});

    const timer = setTimeout(
        () =>
            rejectConnected(
                new Error(`Timed out after ${timeoutMs}ms waiting for node ${nodeId} to reach Connected state`),
            ),
        timeoutMs,
    );

    try {
        const node = await commissioningController.connectNode(nodeId, {
            stateInformationCallback: (_peerNodeId, info) => {
                if (info === NodeStateInformation.Connected) {
                    resolveConnected();
                }
            },
        });
        await connected;
        return node;
    } finally {
        clearTimeout(timer);
    }
}

async function connectOrRecommission(commissioningController, pairingCode) {
    const commissionedNodes = commissioningController.getCommissionedNodes();
    if (commissionedNodes.length > 0) {
        const nodeId = commissionedNodes[0];
        try {
            console.log(`Reusing already-commissioned node ${nodeId}`);
            return await connectAndVerify(commissioningController, nodeId);
        } catch (error) {
            console.log(
                `Could not reconnect to node ${nodeId} (${error?.message ?? error}). The device likely no ` +
                    `longer trusts our fabric (factory reset, or its fabric table was otherwise cleared). ` +
                    `Forgetting the stale node and re-commissioning from scratch.`,
            );
            // false = don't attempt to decommission on the device itself; it already doesn't
            // trust us, so that call would just fail too. This only cleans up local state.
            await commissioningController.removeNode(nodeId, false);
        }
    }

    return commission(commissioningController, pairingCode);
}

async function main() {
    // Parse first and fail fast on bad args, before touching the network/commissioning at all.
    const { endpointId, lowerTimeMs, riseTimeMs, channel, remoteId, numBlinds, pairingCode } = parseArgs(
        process.argv.slice(2),
    );

    const environment = Environment.default;

    const commissioningController = new CommissioningController({
        environment: { environment, id: "blind-config-writer" },
        autoConnect: false,
        // This is the one field Home Assistant shows that holds arbitrary custom text.
        // The separate "Test Vendor" text HA also shows comes from adminVendorId (default
        // 0xFFF1, one of CSA's reserved test-vendor IDs) resolved against CSA's vendor
        // registry — that's an external lookup on a spec-constrained numeric ID (matter.js
        // only allows 0-0xFFF4), not a string we can set, so it can't be changed to custom
        // text without either being invalid or falsely resolving to a real vendor's name.
        adminFabricLabel: "Configuration Script",
    });
    await commissioningController.start();

    // try/finally so a failure partway through (bad pairing code, a write getting rejected,
    // etc.) still closes storage cleanly instead of leaving the same kind of orphaned lock
    // we hit earlier — that fix only covered the success path.
    try {
        const node = await connectOrRecommission(commissioningController, pairingCode);

        // adminFabricLabel above only takes effect for a fresh commissionNode() call, not
        // for a reused/already-commissioned node — push it live too so an already-paired
        // fabric (e.g. one commissioned before this label existed) picks it up without
        // needing another re-pair. Safe/cheap to call every run: it's a no-op if the label
        // set at commissioning time already matches.
        await commissioningController.updateFabricLabel("Configuration Script");

        const interactionClient = await node.getInteractionClient();

        // numBlinds is listed last: it triggers a reboot ~2s after landing, so any per-blind
        // writes above should already be on their way to the device before that happens.
        const writes = [
            { value: lowerTimeMs, attribute: LowerTimeMs, endpointId, clusterId: BLIND_CONFIG_CLUSTER_ID },
            { value: riseTimeMs, attribute: RiseTimeMs, endpointId, clusterId: BLIND_CONFIG_CLUSTER_ID },
            { value: channel, attribute: Channel, endpointId, clusterId: BLIND_CONFIG_CLUSTER_ID },
            { value: remoteId, attribute: RemoteId, endpointId, clusterId: BLIND_CONFIG_CLUSTER_ID },
            { value: numBlinds, attribute: NumBlinds, endpointId: 0, clusterId: SYSTEM_CONFIG_CLUSTER_ID },
        ];
        for (const { value, attribute, endpointId: writeEndpointId, clusterId } of writes) {
            if (value === undefined) {
                continue;
            }
            await interactionClient.setAttribute({
                attributeData: {
                    endpointId: EndpointNumber(writeEndpointId),
                    clusterId: ClusterId(clusterId),
                    attribute,
                    value,
                },
            });
            console.log(`Wrote ${attribute.name} = ${value} (endpoint ${writeEndpointId})`);
        }
    } finally {
        await commissioningController.close();
    }
}

main()
    .then(() => process.exit(0))
    .catch(err => {
        console.error(err);
        process.exit(1);
    });
