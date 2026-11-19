#!/usr/bin/env python3
"""Build the rover's MediaPipe legacy wheel from a verified PyPI wheel."""

import argparse
import base64
import copy
import csv
import hashlib
import io
import platform
import sys
import zipfile
from pathlib import Path


UPSTREAM = "0.10.15"
LOCAL = "0.10.15+rover.1"
PROTOBUF = "6.33.6"
WHEEL_SHA256 = {
    "aarch64": "9f20086c79c4605cc552f95252dee19891990acbeb348fcba03acda0bfebfd0f",
    "x86_64": "f35cd73c0f3a19c7b8c58cee771d396abbb16991761cbb73459c9ed73d51652a",
}
PACKET_GETTER = "mediapipe/python/packet_getter.py"
SOLUTION_BASE = "mediapipe/python/solution_base.py"
PYTHON_INIT = "mediapipe/python/__init__.py"


def replace_exact(data, before, after, count=1):
    if data.count(before) != count:
        raise ValueError(f"Unexpected upstream content for {before!r}")
    return data.replace(before, after)


def patch_packet_getter(data):
    # Backport MediaPipe commit 58cd91d5083e7cc1b4e19f4cc705943ecd8f0ef5.
    data = replace_exact(
        data,
        b"from google.protobuf import message\n",
        b"from google.protobuf import message\nfrom google.protobuf import message_factory\n",
    )
    return replace_exact(
        data,
        b"symbol_database.Default().GetPrototype(descriptor)",
        b"message_factory.GetMessageClass(descriptor)",
        count=2,
    )


def patch_solution_base(data):
    data = replace_exact(data, b"import os\n", b"import os\nimport tempfile\n")
    # The legacy binding stringifies proto objects before parsing them.
    # Keep Any fields byte-exact across Python and native Protobuf versions.
    return replace_exact(
        data,
        b"    self._graph = calculator_graph.CalculatorGraph(\n"
        b"        graph_config=canonical_graph_config_proto)\n",
        b"    with tempfile.NamedTemporaryFile(suffix='.binarypb') as binary_graph:\n"
        b"      binary_graph.write(canonical_graph_config_proto.SerializeToString())\n"
        b"      binary_graph.flush()\n"
        b"      self._graph = calculator_graph.CalculatorGraph(\n"
        b"          binary_graph_path=binary_graph.name)\n",
    )



def patch_native_import(data):
    # The pinned C++ bridge caches a bound factory method when it imports.
    # Give it the supported lookup only during that import, then remove it.
    original = b"from mediapipe.python._framework_bindings import model_ckpt_util\n"
    replacement = b"""from google.protobuf import message_factory as _message_factory


def _load_model_ckpt_util():
  factory = _message_factory.MessageFactory
  added_adapter = not hasattr(factory, 'GetPrototype')

  def get_prototype(self, descriptor):
    return _message_factory.GetMessageClass(descriptor)

  if added_adapter:
    factory.GetPrototype = get_prototype
  try:
    from mediapipe.python._framework_bindings import model_ckpt_util
    return model_ckpt_util
  finally:
    if added_adapter:
      del factory.GetPrototype


model_ckpt_util = _load_model_ckpt_util()
del _load_model_ckpt_util
"""
    return replace_exact(data, original, replacement)


def patch_metadata(data):
    data = replace_exact(data, f"Version: {UPSTREAM}\n".encode(), f"Version: {LOCAL}\n".encode())
    requirement = b"Requires-Dist: protobuf<5,>=4.25.3\n"
    arm64_requirement = b"Requires-Dist: protobuf <5,>=4.25.3\n"
    if arm64_requirement in data:
        requirement = arm64_requirement
    return replace_exact(data, requirement, f"Requires-Dist: protobuf =={PROTOBUF}\n".encode())


def build(source, target):
    upstream_info = f"mediapipe-{UPSTREAM}.dist-info/"
    local_info = f"mediapipe-{LOCAL}.dist-info/"
    metadata = upstream_info + "METADATA"
    record = upstream_info + "RECORD"
    records = io.StringIO(newline="")
    writer = csv.writer(records, lineterminator="\n")
    with zipfile.ZipFile(source) as upstream, zipfile.ZipFile(target, "w") as result:
        original_record = upstream.getinfo(record)
        for path in (metadata, PACKET_GETTER, SOLUTION_BASE, PYTHON_INIT):
            upstream.getinfo(path)
        for member in upstream.infolist():
            name = member.filename
            if member.is_dir() or name == record:
                continue
            data = upstream.read(member)
            local = copy.copy(member)
            if name.startswith(upstream_info):
                local.filename = local_info + name.removeprefix(upstream_info)
            if name == metadata:
                data = patch_metadata(data)
            if name == PACKET_GETTER:
                data = patch_packet_getter(data)
            if name == SOLUTION_BASE:
                data = patch_solution_base(data)
            if name == PYTHON_INIT:
                data = patch_native_import(data)
            result.writestr(local, data)
            digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=").decode()
            writer.writerow((local.filename, f"sha256={digest}", len(data)))
        new_record = copy.copy(original_record)
        new_record.filename = local_info + "RECORD"
        writer.writerow((new_record.filename, "", ""))
        result.writestr(new_record, records.getvalue().encode())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    architecture = platform.machine()
    if platform.system() != "Linux" or architecture not in WHEEL_SHA256 or sys.version_info[:2] != (3, 11):
        parser.error("the rover MediaPipe wheel requires CPython 3.11 on Linux ARM64 or x86-64")
    checksum = WHEEL_SHA256[architecture]
    platform_tag = f"cp311-cp311-manylinux_2_17_{architecture}.manylinux2014_{architecture}"
    filename = f"mediapipe-{UPSTREAM}-{platform_tag}.whl"
    target = args.directory / f"mediapipe-{LOCAL}-{platform_tag}.whl"
    source = args.directory / filename
    with source.open("rb") as handle:
        if hashlib.file_digest(handle, "sha256").hexdigest() != checksum:
            raise ValueError("The upstream MediaPipe wheel did not match its pinned SHA-256")
    build(source, target)
    source.unlink()
    print(target)


if __name__ == "__main__":
    main()
