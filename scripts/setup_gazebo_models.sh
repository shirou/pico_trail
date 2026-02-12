#!/bin/bash
# Generate per-vehicle r1_rover models with unique fdm_port_in values.
# Injects ArduPilotPlugin into model.sdf if not already present.
# Usage: ./scripts/setup_gazebo_models.sh [COUNT] [BASE_PORT] [STRIDE]

set -e

COUNT=${1:-3}
BASE_PORT=${2:-9002}
STRIDE=${3:-10}

MODELS_SRC="${GZ_SIM_RESOURCE_PATH%%:*}"  # first path entry
if [ ! -d "$MODELS_SRC/r1_rover" ]; then
    echo "Error: r1_rover model not found in $MODELS_SRC" >&2
    exit 1
fi

for i in $(seq 1 "$COUNT"); do
    PORT=$(( BASE_PORT + (i - 1) * STRIDE ))
    DEST="$MODELS_SRC/r1_rover_$i"

    rm -rf "$DEST"
    cp -r "$MODELS_SRC/r1_rover" "$DEST"

    # Update model.config name
    sed -i "s|<name>r1_rover</name>|<name>r1_rover_$i</name>|" "$DEST/model.config"

    # If model already has ArduPilotPlugin, update port and disable lock_step
    if grep -q "ArduPilotPlugin" "$DEST/model.sdf"; then
        sed -i "s|<fdm_port_in>[0-9]*</fdm_port_in>|<fdm_port_in>${PORT}</fdm_port_in>|" "$DEST/model.sdf"
        sed -i "s|<lock_step>[0-9]*</lock_step>|<lock_step>0</lock_step>|" "$DEST/model.sdf"
        echo "r1_rover_$i: updated existing plugin (fdm_port_in=$PORT, lock_step=0)"
    else
        # Inject ArduPilotPlugin before closing </model> tag
        cat > /tmp/plugin_block.xml <<XMLEOF
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>${PORT}</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>0</lock_step>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <imuName>base_link::imu_sensor</imuName>
      <control channel="0">
        <jointName>motor_0</jointName>
        <useForce>1</useForce>
        <multiplier>46.3</multiplier>
        <offset>-0.5</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.2</p_gain>
        <i_gain>0.06</i_gain>
        <d_gain>0.0001</d_gain>
        <i_max>1</i_max>
        <i_min>-1</i_min>
        <cmd_max>-1.0</cmd_max>
        <cmd_min>0.0</cmd_min>
      </control>
      <control channel="0">
        <jointName>motor_1</jointName>
        <useForce>1</useForce>
        <multiplier>46.3</multiplier>
        <offset>-0.5</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.2</p_gain>
        <i_gain>0.06</i_gain>
        <d_gain>0.0001</d_gain>
        <i_max>1</i_max>
        <i_min>-1</i_min>
        <cmd_max>-1.0</cmd_max>
        <cmd_min>0.0</cmd_min>
      </control>
      <control channel="2">
        <jointName>motor_2</jointName>
        <useForce>1</useForce>
        <multiplier>-46.3</multiplier>
        <offset>-0.5</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.2</p_gain>
        <i_gain>0.06</i_gain>
        <d_gain>0.0001</d_gain>
        <i_max>1</i_max>
        <i_min>-1</i_min>
        <cmd_max>-1.0</cmd_max>
        <cmd_min>0.0</cmd_min>
      </control>
      <control channel="2">
        <jointName>motor_3</jointName>
        <useForce>1</useForce>
        <multiplier>-46.3</multiplier>
        <offset>-0.5</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.2</p_gain>
        <i_gain>0.06</i_gain>
        <d_gain>0.0001</d_gain>
        <i_max>1</i_max>
        <i_min>-1</i_min>
        <cmd_max>-1.0</cmd_max>
        <cmd_min>0.0</cmd_min>
      </control>
    </plugin>
XMLEOF
        # Insert plugin block before </model>
        sed -i "/<\/model>/{ r /tmp/plugin_block.xml
        }" "$DEST/model.sdf"
        rm -f /tmp/plugin_block.xml
        echo "r1_rover_$i: injected ArduPilotPlugin (fdm_port_in=$PORT)"
    fi

    # Verify
    if grep -q "ArduPilotPlugin" "$DEST/model.sdf"; then
        echo "  OK: plugin present in model.sdf"
    else
        echo "  ERROR: plugin injection failed!" >&2
        exit 1
    fi
done
