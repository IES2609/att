#!/usr/bin/env python3
"""Fetch and decode nRF Cloud d2c messages for the custom test_data payload.

Usage:
  export NRF_CLOUD_DEVICE_ID=your_device_id
  export NRF_CLOUD_API_KEY=your_api_key
  python3 fetch_nrfcloud_d2c_test_data.py --minutes 5 --dump-raw raw_messages.json -o decoded.json
"""

import argparse
import base64
import datetime as dt
import binascii
import json
import os
import re
import struct
import sys
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict, Iterable, List, Optional

API_BASE = "https://api.nrfcloud.com/v1/messages"
ENV_DEVICE_ID = "NRF_CLOUD_DEVICE_ID"
ENV_API_KEY = "NRF_CLOUD_API_KEY"
TEST_DATA_STRUCT = struct.Struct("<Q9d")  # uint64_t + 9 doubles, little-endian
PAYLOAD_HEADER_STRUCT = struct.Struct("<II")  # uint32_t count + uint32_t sequence
MIN_VALID_TS_MS = 1577836800000  # 2020-01-01T00:00:00Z
MAX_VALID_TS_MS = 4102444800000  # 2100-01-01T00:00:00Z


def _iso8601_z(ts: dt.datetime) -> str:
    return ts.replace(microsecond=(ts.microsecond // 1000) * 1000).isoformat().replace(
        "+00:00", "Z"
    )


def _utc_now() -> dt.datetime:
    return dt.datetime.now(tz=dt.timezone.utc)


def fetch_messages(device_id: str, api_key: str, minutes: int = 5) -> Any:
    # Query only a recent time window using start/end ISO-8601 timestamps.
    minutes = max(1, minutes)
    end_ts = _utc_now()
    start_ts = end_ts - dt.timedelta(minutes=minutes)

    query = urllib.parse.urlencode(
        {
            "deviceId": device_id,
            "type": "d2c",
            "start": _iso8601_z(start_ts),
            "end": _iso8601_z(end_ts),
            "pageSort": "desc",
        }
    )
    url = f"{API_BASE}?{query}"

    req = urllib.request.Request(
        url,
        headers={
            "Authorization": f"Bearer {api_key}",
            "Accept": "application/json",
        },
        method="GET",
    )

    with urllib.request.urlopen(req, timeout=20) as resp:
        return json.loads(resp.read().decode("utf-8"))


def iter_message_objects(payload: Any) -> Iterable[Dict[str, Any]]:
    if isinstance(payload, list):
        for item in payload:
            if isinstance(item, dict):
                yield item
        return

    if not isinstance(payload, dict):
        return

    for key in ("items", "messages", "data"):
        val = payload.get(key)
        if isinstance(val, list):
            for item in val:
                if isinstance(item, dict):
                    yield item
            return

    yield payload


def _possible_payload_strings(msg: Dict[str, Any]) -> Iterable[str]:
    candidates = [
        msg.get("message"),
        msg.get("payload"),
        msg.get("data"),
        msg.get("value"),
    ]

    nested = msg.get("message")
    if isinstance(nested, dict):
        candidates.extend(
            [
                nested.get("data"),
                nested.get("payload"),
                nested.get("value"),
                nested.get("bytes"),
                nested.get("raw"),
                nested.get("body"),
            ]
        )

    for item in candidates:
        if isinstance(item, str) and item.strip():
            yield item.strip()


def _possible_payload_bytes(msg: Dict[str, Any]) -> Iterable[bytes]:
    candidates = [
        msg.get("bytes"),
        msg.get("raw"),
        msg.get("body"),
    ]

    nested = msg.get("message")
    if isinstance(nested, dict):
        candidates.extend(
            [
                nested.get("bytes"),
                nested.get("raw"),
                nested.get("body"),
                nested.get("data"),
                nested.get("payload"),
                nested.get("value"),
            ]
        )

    for item in candidates:
        if isinstance(item, (list, tuple)) and item and all(
            isinstance(x, int) and 0 <= x <= 255 for x in item
        ):
            yield bytes(item)


def _decode_candidate_string(s: str) -> Iterable[bytes]:
    # Try strict base64 first.
    try:
        yield base64.b64decode(s, validate=True)
    except Exception:
        pass

    # Then base64url with implicit padding.
    try:
        padded = s + "=" * ((4 - len(s) % 4) % 4)
        yield base64.urlsafe_b64decode(padded)
    except Exception:
        pass

    # Finally hex encodings (plain or with separators/0x prefix).
    cleaned = re.sub(r"[^0-9a-fA-F]", "", s.replace("0x", "").replace("0X", ""))
    if cleaned and len(cleaned) % 2 == 0:
        try:
            yield binascii.unhexlify(cleaned)
        except Exception:
            pass


def _decode_test_data_record(raw: bytes) -> Dict[str, Any]:
    values = TEST_DATA_STRUCT.unpack(raw)
    ts_ms = int(values[0])
    timestamp_valid = MIN_VALID_TS_MS <= ts_ms <= MAX_VALID_TS_MS
    timestamp_iso = None
    try:
        if timestamp_valid:
            timestamp_iso = dt.datetime.fromtimestamp(ts_ms / 1000, tz=dt.timezone.utc).isoformat()
    except (ValueError, OSError, OverflowError):
        timestamp_valid = False
        timestamp_iso = None

    return {
        "timestamp_ms": ts_ms,
        "timestamp_iso": timestamp_iso,
        "timestamp_valid": timestamp_valid,
        "ax1": {"x": values[1], "y": values[2], "z": values[3]},
        "ax2": {"x": values[4], "y": values[5], "z": values[6]},
        "gyr": {"x": values[7], "y": values[8], "z": values[9]},
    }


def _decode_records_from_raw(raw: bytes) -> Dict[str, Any]:
    record_size = TEST_DATA_STRUCT.size

    # Preferred format from cloud.c: payload_data { count, sequence, data[count] }.
    if len(raw) >= PAYLOAD_HEADER_STRUCT.size + record_size:
        count, sequence = PAYLOAD_HEADER_STRUCT.unpack(raw[: PAYLOAD_HEADER_STRUCT.size])
        data_offset = PAYLOAD_HEADER_STRUCT.size
        payload_data_bytes = len(raw) - data_offset

        # Strict layout check: after header, payload must be a whole number of records.
        if payload_data_bytes % record_size != 0:
            count = 0

        available_records = payload_data_bytes // record_size

        # Treat count as valid only when it is plausible for this payload size.
        if count > 0 and count == available_records:
            records = []
            for i in range(count):
                start = data_offset + (i * record_size)
                end = start + record_size
                records.append(_decode_test_data_record(raw[start:end]))

            return {
                "records": records,
                "payload": {
                    "format": "payload_data",
                    "count": count,
                    "sequence": sequence,
                    "raw_payload_length": len(raw),
                },
            }

    # Backward-compatible fallback: raw contains only test_data records.
    if len(raw) >= record_size:
        record_count = len(raw) // record_size
        records = []
        for i in range(record_count):
            start = i * record_size
            end = start + record_size
            records.append(_decode_test_data_record(raw[start:end]))

        return {
            "records": records,
            "payload": {
                "format": "record_array",
                "count": record_count,
                "sequence": None,
                "raw_payload_length": len(raw),
            },
        }

    return {
        "records": [],
        "payload": {
            "format": "unknown",
            "count": 0,
            "sequence": None,
            "raw_payload_length": len(raw),
        },
    }


def decode_test_data_from_message(msg: Dict[str, Any]) -> Dict[str, Any]:
    raw = None

    for candidate in _possible_payload_bytes(msg):
        if len(candidate) >= TEST_DATA_STRUCT.size:
            raw = candidate
            break

    for s in _possible_payload_strings(msg):
        for candidate in _decode_candidate_string(s):
            if len(candidate) >= TEST_DATA_STRUCT.size:
                raw = candidate
                break
        if raw is not None:
            break

    if raw is None:
        return {
            "records": [],
            "payload": {
                "format": "missing",
                "count": 0,
                "sequence": None,
                "raw_payload_length": 0,
            },
        }

    return _decode_records_from_raw(raw)


def extract_message_metadata(msg: Dict[str, Any]) -> Dict[str, Any]:
    top_level_keys = (
        "id",
        "messageId",
        "deviceId",
        "appId",
        "type",
        "subType",
        "topic",
        "qos",
        "dup",
        "retain",
        "ts",
        "timestamp",
        "receivedAt",
        "createdAt",
    )

    metadata: Dict[str, Any] = {}
    for key in top_level_keys:
        if key in msg:
            metadata[key] = msg[key]

    nested = msg.get("message")
    if isinstance(nested, dict):
        nested_meta = {
            k: nested[k]
            for k in (
                "id",
                "messageId",
                "type",
                "subType",
                "topic",
                "qos",
                "ts",
                "timestamp",
            )
            if k in nested
        }
        if nested_meta:
            metadata["message"] = nested_meta

    return metadata


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fetch and decode the last N d2c messages from nRF Cloud."
    )
    parser.add_argument(
        "-o",
        "--output",
        default="nrf_cloud_messages.json",
        help="Output JSON file path (default: nrf_cloud_messages.json)",
    )
    parser.add_argument(
        "-n",
        "--minutes",
        type=int,
        default=5,
        help="Fetch messages from the last N minutes (default: 5)",
    )
    parser.add_argument(
        "--dump-raw",
        default=None,
        help="Optional file path to save raw API response for debugging",
    )
    args = parser.parse_args()

    device_id = os.getenv(ENV_DEVICE_ID)
    api_key = os.getenv(ENV_API_KEY)

    if not device_id or not api_key:
        print(
            f"Missing env vars. Set {ENV_DEVICE_ID} and {ENV_API_KEY}.",
            file=sys.stderr,
        )
        return 2

    try:
        payload = fetch_messages(device_id, api_key, minutes=args.minutes)
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        print(f"HTTP {e.code}: {body}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Request failed: {e}", file=sys.stderr)
        return 1

    if args.dump_raw:
        try:
            with open(args.dump_raw, "w") as f:
                json.dump(payload, f, indent=2)
        except IOError as e:
            print(f"Failed to write raw response to {args.dump_raw}: {e}", file=sys.stderr)
            return 1

    raw_messages = list(iter_message_objects(payload))
    
    # --- NYE VARIABLER FOR ANALYSE ---
    total_raw_bytes_received = 0
    total_bytes_attempted_decoded = 0
    # ---------------------------------

    decoded: List[Dict[str, Any]] = []
    decoded_messages = 0
    
    for msg in raw_messages:
        # Finn rådata-lengden i denne spesifikke meldingen før dekoding for analyse
        msg_raw_bytes = 0
        for b_cand in _possible_payload_bytes(msg):
            msg_raw_bytes = max(msg_raw_bytes, len(b_cand))
        for s_cand in _possible_payload_strings(msg):
            for decoded_cand in _decode_candidate_string(s_cand):
                msg_raw_bytes = max(msg_raw_bytes, len(decoded_cand))
        
        total_raw_bytes_received += msg_raw_bytes

        # Dekoding
        decoded_msg = decode_test_data_from_message(msg)
        total_bytes_attempted_decoded += decoded_msg["payload"]["raw_payload_length"]
        
        decoded_payloads = decoded_msg["records"]
        payload_info = decoded_msg["payload"]

        if decoded_payloads:
            decoded_messages += 1
            for idx, decoded_payload in enumerate(decoded_payloads):
                decoded.append(
                    {
                        "metadata": {
                            **extract_message_metadata(msg),
                            "payload_record_index": idx,
                            "payload_record_count": len(decoded_payloads),
                            "payload_format": payload_info["format"],
                            "payload_count": payload_info["count"],
                            "payload_sequence": payload_info["sequence"],
                            "raw_payload_length": payload_info["raw_payload_length"],
                        },
                        "decoded": decoded_payload,
                    }
                )

    # --- UTSKRIFT AV ANALYSE ---
    print("\n" + "="*40)
    print(" RAW DATA ANALYSE")
    print("-"*40)
    print(f"Meldinger hentet:          {len(raw_messages)}")
    print(f"Totalt mottatt fra API:    {total_raw_bytes_received} bytes ({total_raw_bytes_received / 1024 / 1024:.2f} MB)")
    print(f"Bytes sendt til dekoder:   {total_bytes_attempted_decoded} bytes")
    print(f"Antall dekodede samples:   {len(decoded)}")
    
    expected_3mb = 3145728 # 3 * 1024 * 1024
    if total_raw_bytes_received < expected_3mb * 0.9:
        print("\n[!] ADVARSEL: Du mottok betydelig mindre enn 3MB.")
        print(f"    Sjekk om '--minutes {args.minutes}' dekker hele tidsrommet for sendingen.")
    elif len(decoded) < (total_bytes_attempted_decoded / 80) * 0.9:
        print("\n[!] INFO: Data er mottatt, men mange bytes ble forkastet under dekoding.")
        print("    Dette skjer ofte hvis 'record_size' (80 bytes) ikke går opp i pakkelengden.")
    print("="*40 + "\n")

    result = {
        "requested_minutes": args.minutes,
        "total_bytes_received": total_raw_bytes_received,
        "fetched_count": len(raw_messages),
        "decoded_messages_count": decoded_messages,
        "decoded_count": len(decoded),
        "samples": decoded,
    }
    
    try:
        with open(args.output, "w") as f:
            json.dump(result, f, indent=2)
        print(f"Lagret resultat til {args.output}")
    except IOError as e:
        print(f"Failed to write to {args.output}: {e}", file=sys.stderr)
        return 1

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
