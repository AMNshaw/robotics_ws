#!/usr/bin/env bash
#
# Download NCLT dataset (velodyne + sensor) and convert to ROS2 bag.
#
# Usage:
#   ./download_and_convert.sh 2013-04-05
#   ./download_and_convert.sh 2012-01-08 --no-imu
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
S3_BASE="https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu"

DATE="${1:?Usage: $0 <date> [--no-imu]}"
shift
EXTRA_ARGS="$*"

RAW_DIR="${SCRIPT_DIR}/raw/nclt"
BAG_DIR="${SCRIPT_DIR}/bags"
DATA_DIR="${RAW_DIR}/${DATE}"

mkdir -p "${RAW_DIR}" "${BAG_DIR}" "${DATA_DIR}"

# ─── Download velodyne ───────────────────────────────────────────────────────
VEL_TAR="${RAW_DIR}/${DATE}_vel.tar.gz"
if [[ ! -f "${DATA_DIR}/velodyne_hits.bin" ]]; then
    echo "[1/4] Downloading velodyne data..."
    wget --continue "${S3_BASE}/velodyne_data/${DATE}_vel.tar.gz" -O "${VEL_TAR}"
    echo "[2/4] Extracting velodyne..."
    tar -xzf "${VEL_TAR}" -C "${RAW_DIR}/"
    # tar creates ${DATE}/velodyne_hits.bin inside RAW_DIR
else
    echo "[1/4] Velodyne already extracted, skipping download."
fi

# ─── Download sensor (IMU) ───────────────────────────────────────────────────
SEN_TAR="${RAW_DIR}/${DATE}_sen.tar.gz"
if [[ ! -f "${DATA_DIR}/ms25.csv" ]] && [[ ! " ${EXTRA_ARGS} " =~ "--no-imu" ]]; then
    echo "[3/4] Downloading sensor data (IMU)..."
    wget --continue "${S3_BASE}/sensor_data/${DATE}_sen.tar.gz" -O "${SEN_TAR}"
    echo "       Extracting sensor data..."
    tar -xzf "${SEN_TAR}" -C "${RAW_DIR}/"
    # Flatten nested dir if needed (tar extracts as ${DATE}/${DATE}/...)
    if [[ -d "${DATA_DIR}/${DATE}" ]]; then
        mv "${DATA_DIR}/${DATE}"/* "${DATA_DIR}/"
        rmdir "${DATA_DIR}/${DATE}"
    fi
else
    echo "[3/4] IMU data already present or skipped."
fi

# ─── Convert to ROS2 bag ────────────────────────────────────────────────────
OUTPUT_BAG="${BAG_DIR}/nclt_${DATE}"
if [[ -d "${OUTPUT_BAG}" ]]; then
    echo "[4/4] Bag ${OUTPUT_BAG} already exists. Delete it to reconvert."
else
    echo "[4/4] Converting to ROS2 bag..."
    python3 "${SCRIPT_DIR}/nclt_to_ros2bag.py" ${EXTRA_ARGS} "${DATA_DIR}/" "${OUTPUT_BAG}"

    # Fix metadata for ROS2 compatibility (rosbags lib format quirks)
    if [[ -f "${OUTPUT_BAG}/metadata.yaml" ]]; then
        sed -i '/type_description_hash:/{N;s/type_description_hash: *\n *\(RIHS01[^ ]*\)/type_description_hash: \1/}' "${OUTPUT_BAG}/metadata.yaml"
        # Replace offered_qos_profiles: [] with ''
        sed -i "s/offered_qos_profiles: \[\]/offered_qos_profiles: ''/g" "${OUTPUT_BAG}/metadata.yaml"
        # Replace custom_data: null
        sed -i "s/custom_data: null/custom_data: {}/g" "${OUTPUT_BAG}/metadata.yaml"
    fi
fi

echo ""
echo "Done! Bag info:"
ros2 bag info "${OUTPUT_BAG}" 2>/dev/null || echo "(ros2 not sourced — bag is at ${OUTPUT_BAG})"
echo ""
echo "Play with:"
echo "  ros2 bag play ${OUTPUT_BAG} --clock"
