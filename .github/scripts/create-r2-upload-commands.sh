#!/usr/bin/env bash

set -eu

for name in CLOUDFLARE_ACCOUNT_ID R2_BUCKET; do
  if [ -z "${!name}" ]; then
    echo "::error::Missing required environment value: $name"
    exit 1
  fi
done

artifact_dir="${RUNNER_TEMP}/firmware-artifacts"
artifact_names=(zephyr.bin zephyr.elf zephyr.hex)
artifacts=()

for name in "${artifact_names[@]}"; do
  artifact="${artifact_dir}/${name}"
  if [ ! -f "$artifact" ]; then
    echo "::error::Firmware artifact not found: $artifact"
    exit 1
  fi
  artifacts+=("$artifact")
done

case "$EVENT_NAME:$REF_TYPE" in
  workflow_dispatch:*)
    object_prefix="snapshot/${REPOSITORY_NAME}/${SHA}"
    ;;
  push:tag)
    object_prefix="${R2_RELEASE_PREFIX%/}/${REF_NAME}/sp-firmware"
    ;;
  *)
    echo "::error::Unsupported publish event: $EVENT_NAME ($REF_TYPE)" >&2
    exit 1
    ;;
esac

{
  echo "commands<<EOF"
  for artifact in "${artifacts[@]}"; do
    object_path="${R2_BUCKET}/${object_prefix}/$(basename "$artifact")"
    printf 'r2 object put %q --file %q --content-type application/octet-stream --remote\n' \
      "$object_path" \
      "$artifact"
  done
  echo "EOF"
} >> "$GITHUB_OUTPUT"

for artifact in "${artifacts[@]}"; do
  object_path="${R2_BUCKET}/${object_prefix}/$(basename "$artifact")"
  echo "Uploading $(basename "$artifact") to r2://$object_path"
done
