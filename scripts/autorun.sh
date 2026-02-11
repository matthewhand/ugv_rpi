#!/bin/bash
if [ -n "$SUDO_USER" ] || [ -n "$SUDO_UID" ]; then
    echo "This script was executed with sudo."
    echo "Use './autorun.sh' instead of 'sudo ./autorun.sh'"
    exit 1
fi

USER_NAME=$(logname)
USER_HOME=$(eval echo "~$USER_NAME")
SYSTEMD_DIR="$USER_HOME/.config/systemd/user"

APP_PATH="$USER_HOME/ugv_rpi/app.py"
PYTHON_BIN="$USER_HOME/ugv_rpi/ugv-env/bin/python"

mkdir -p "$SYSTEMD_DIR"

APP_SERVICE="$SYSTEMD_DIR/ugv-app.service"
cat > "$APP_SERVICE" <<EOL
[Unit]
Description=UGV Python App
After=sound.target pipewire.service
Wants=pipewire.service

[Service]
ExecStart=/bin/bash -c "$USER_HOME/ugv_rpi/ugv-env/bin/python -u $USER_HOME/ugv_rpi/app.py >> $USER_HOME/ugv_rpi/ugv-app.log 2>&1"
Restart=always
Environment=XDG_RUNTIME_DIR=/run/user/%U
WorkingDirectory=$USER_HOME/ugv_rpi

[Install]
WantedBy=default.target
EOL

JUPYTER_SERVICE="$SYSTEMD_DIR/ugv-jupyter.service"
cat > "$JUPYTER_SERVICE" <<EOL
[Unit]
Description=UGV Jupyter Notebook

[Service]
ExecStart=/bin/bash -c "$USER_HOME/ugv_rpi/scripts/start_jupyter.sh >> $USER_HOME/ugv_rpi/ugv-jupyter.log 2>&1"
Restart=always
Environment=XDG_RUNTIME_DIR=/run/user/%U
WorkingDirectory=$USER_HOME

[Install]
WantedBy=default.target
EOL

systemctl --user daemon-reload
systemctl --user enable ugv-app.service
systemctl --user enable ugv-jupyter.service
sudo loginctl enable-linger $USER_NAME

export PATH=$HOME/.local/bin:$PATH
source "$USER_HOME/ugv_rpi/ugv-env/bin/activate"
CONFIG_FILE="$USER_HOME/.jupyter/jupyter_notebook_config.py"
if [ ! -f "$CONFIG_FILE" ]; then
    jupyter notebook --generate-config
fi

grep -q "c.NotebookApp.token" "$CONFIG_FILE" || echo "c.NotebookApp.token = ''" >> "$CONFIG_FILE"
grep -q "c.NotebookApp.password" "$CONFIG_FILE" || echo "c.NotebookApp.password = ''" >> "$CONFIG_FILE"

echo "Setup complete. You can start services with:"
echo "systemctl --user start ugv-app.service"
echo "systemctl --user start ugv-jupyter.service"
echo "Logs: journalctl --user -u ugv-app.service -f"
echo "      journalctl --user -u ugv-jupyter.service -f"
