#!/usr/bin/env python3
"""Manually test one PNOZ virtual input and observe physical outputs O0..O3."""

import argparse
import fcntl
import socket
import struct
import sys
import time


OUTPUT_NAMES = (
    "O0 motor_sto_01sr",
    "O1 motor_sto_02sr",
    "O2 charge_port_on",
    "O3 traction_motor_power_on",
)
OUTPUT_BASE = 0x4020
OUTPUT_COUNT = len(OUTPUT_NAMES)
VIRTUAL_INPUT_COUNT = 128


class ModbusError(RuntimeError):
    pass


class ModbusTcpClient:
    def __init__(self, host, port, unit_id, timeout):
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self.sock = None
        self.transaction_id = 0

    def connect(self):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), self.timeout)
        self.sock.settimeout(self.timeout)

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def _recv_exact(self, size):
        chunks = []
        remaining = size
        while remaining:
            chunk = self.sock.recv(remaining)
            if not chunk:
                raise ModbusError("connection closed while receiving a response")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _transact(self, pdu):
        if self.sock is None:
            self.connect()

        self.transaction_id = (self.transaction_id + 1) & 0xFFFF
        request = struct.pack(
            ">HHHB", self.transaction_id, 0, len(pdu) + 1, self.unit_id
        ) + pdu
        try:
            self.sock.sendall(request)
            header = self._recv_exact(7)
            transaction_id, protocol_id, length, unit_id = struct.unpack(">HHHB", header)
            if transaction_id != self.transaction_id:
                raise ModbusError("transaction ID mismatch")
            if protocol_id != 0 or length < 2 or unit_id != self.unit_id:
                raise ModbusError("invalid Modbus/TCP response header")
            response = self._recv_exact(length - 1)
        except (OSError, ModbusError):
            self.close()
            raise

        if response[0] == (pdu[0] | 0x80):
            code = response[1] if len(response) > 1 else -1
            raise ModbusError("Modbus exception for FC{:02d}: code {}".format(pdu[0], code))
        if response[0] != pdu[0]:
            raise ModbusError("unexpected Modbus function code {}".format(response[0]))
        return response

    def read_bits(self, function_code, address, quantity):
        if function_code not in (1, 2):
            raise ValueError("function_code must be FC01 or FC02")
        response = self._transact(
            struct.pack(">BHH", function_code, address, quantity)
        )
        expected_bytes = (quantity + 7) // 8
        if len(response) != expected_bytes + 2 or response[1] != expected_bytes:
            raise ModbusError("malformed FC{:02d} response".format(function_code))
        return [
            bool(response[2 + bit // 8] & (1 << (bit % 8)))
            for bit in range(quantity)
        ]

    def read_virtual_inputs(self):
        return self.read_bits(1, 0, VIRTUAL_INPUT_COUNT)

    def read_outputs(self):
        return self.read_bits(2, OUTPUT_BASE, OUTPUT_COUNT)

    def write_virtual_input(self, index, value):
        encoded_value = 0xFF00 if value else 0x0000
        request = struct.pack(">BHH", 5, index, encoded_value)
        response = self._transact(request)
        if response != request:
            raise ModbusError("malformed FC05 echo response")


class DeviceLock:
    def __init__(self, host, port):
        endpoint = "{}_{}".format(host, port)
        endpoint = "".join(character if character.isalnum() else "_" for character in endpoint)
        self.path = "/tmp/safety_io_driver_{}.lock".format(endpoint)
        self.handle = None

    def __enter__(self):
        self.handle = open(self.path, "a+")
        try:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.handle.close()
            self.handle = None
            raise RuntimeError(
                "another local process is using this PLC; stop safety_io_node first "
                "({})".format(self.path)
            ) from exc
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.handle is not None:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
            self.handle.close()


def format_outputs(values):
    return ", ".join(
        "{}={}".format(name, "ON" if value else "OFF")
        for name, value in zip(OUTPUT_NAMES, values)
    )


def confirm_test(candidate):
    if not sys.stdin.isatty():
        raise RuntimeError("interactive terminal required for safety confirmation")
    expected = "TEST i{}".format(candidate)
    print("\nWARNING")
    print("  - Motor/charger output loads must be physically isolated.")
    print("  - The PLC logic must remain powered so O0..O3 can be observed.")
    print("  - Only virtual input i{} will be written; physical I/O addresses are read-only.".format(candidate))
    answer = input("Type '{}' to continue: ".format(expected)).strip()
    if answer != expected:
        raise RuntimeError("confirmation did not match; no output was written")


def clear_candidate(client, candidate):
    errors = []
    for attempt in range(2):
        try:
            if attempt:
                client.connect()
            client.write_virtual_input(candidate, False)
            return None
        except (OSError, ModbusError) as exc:
            errors.append(str(exc))
            client.close()
    return "; ".join(errors)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Test exactly one PNOZ virtual input and compare physical O0..O3. "
            "Run only after output loads are physically isolated."
        )
    )
    parser.add_argument("--candidate", type=int, required=True, help="virtual input number, 0..127")
    parser.add_argument("--ip", default="192.168.100.103")
    parser.add_argument("--port", type=int, default=502)
    parser.add_argument("--unit-id", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--settle", type=float, default=0.5, help="seconds to wait after ON")
    parser.add_argument(
        "--restore-wait", type=float, default=0.5, help="seconds to wait after OFF"
    )
    return parser.parse_args()


def validate_args(args):
    if not 0 <= args.candidate < VIRTUAL_INPUT_COUNT:
        raise ValueError("--candidate must be in [0, 127]")
    if not 1 <= args.port <= 65535:
        raise ValueError("--port must be in [1, 65535]")
    if not 0 <= args.unit_id <= 255:
        raise ValueError("--unit-id must be in [0, 255]")
    if args.timeout <= 0 or args.settle < 0.1 or args.restore_wait < 0.1:
        raise ValueError(
            "--timeout must be positive; --settle and --restore-wait must be at least 0.1"
        )


def run(args):
    validate_args(args)
    client = ModbusTcpClient(args.ip, args.port, args.unit_id, args.timeout)

    with DeviceLock(args.ip, args.port):
        try:
            client.connect()
            virtual_inputs = client.read_virtual_inputs()
            baseline = client.read_outputs()
            active_inputs = [index for index, value in enumerate(virtual_inputs) if value]

            print("PLC: {}:{} unit={}".format(args.ip, args.port, args.unit_id))
            print("Active virtual inputs: {}".format(
                ", ".join("i{}".format(index) for index in active_inputs) or "none"
            ))
            print("Baseline outputs: {}".format(format_outputs(baseline)))

            if virtual_inputs[args.candidate]:
                raise RuntimeError(
                    "i{} is already ON; refusing to overwrite an active command".format(args.candidate)
                )
            if baseline[2]:
                raise RuntimeError("O2 is already ON; this candidate cannot be identified from a transition")

            confirm_test(args.candidate)

            # Re-check immediately before the write so a state change during the
            # confirmation prompt cannot invalidate the baseline.
            if client.read_virtual_inputs()[args.candidate]:
                raise RuntimeError("i{} became ON; test aborted".format(args.candidate))
            before_write = client.read_outputs()
            if before_write != baseline:
                raise RuntimeError(
                    "physical outputs changed before the test: {}".format(format_outputs(before_write))
                )

            test_outputs = None
            cleanup_error = None
            try:
                print("Writing i{}=ON for {:.3f} s ...".format(args.candidate, args.settle))
                client.write_virtual_input(args.candidate, True)
                time.sleep(args.settle)
                test_outputs = client.read_outputs()
            finally:
                cleanup_error = clear_candidate(client, args.candidate)

            if cleanup_error:
                raise RuntimeError(
                    "CRITICAL: failed to restore i{}=OFF: {}".format(args.candidate, cleanup_error)
                )

            time.sleep(args.restore_wait)
            restored_virtual = client.read_virtual_inputs()[args.candidate]
            restored_outputs = client.read_outputs()
            if restored_virtual:
                raise RuntimeError("CRITICAL: i{} still reads ON after cleanup".format(args.candidate))

            print("Test outputs:     {}".format(format_outputs(test_outputs)))
            print("Restored outputs: {}".format(format_outputs(restored_outputs)))
            if restored_outputs != baseline:
                raise RuntimeError(
                    "CRITICAL: outputs did not return to the baseline after i{}=OFF".format(
                        args.candidate
                    )
                )

            changed = [index for index in range(OUTPUT_COUNT) if test_outputs[index] != baseline[index]]
            unexpected = [index for index in changed if index != 2]
            if unexpected:
                names = ", ".join(OUTPUT_NAMES[index] for index in unexpected)
                raise RuntimeError(
                    "UNSAFE/UNEXPECTED: i{} changed {}; do not use it for charging".format(
                        args.candidate, names
                    )
                )
            if not baseline[2] and test_outputs[2]:
                print("RESULT: i{} is a likely charging command candidate (O2 OFF -> ON).".format(args.candidate))
                return 0

            print(
                "RESULT: inconclusive; i{} did not turn O2 ON. "
                "The input may be unused or charging interlocks may not be satisfied.".format(args.candidate)
            )
            return 2
        finally:
            client.close()


def main():
    try:
        return run(parse_args())
    except KeyboardInterrupt:
        print("\nInterrupted; cleanup was attempted.", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, ModbusError, OSError) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
