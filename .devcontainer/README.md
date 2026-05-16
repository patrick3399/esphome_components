# Devcontainer Notes

## ESPHome Language Server

The ESPHome VSCode extension provides YAML validation, completion, and hover
help. It validates through ESPHome itself, so the devcontainer installs ESPHome
locally and configures the extension to use the local validator.

Expected setup:

- Extension: `ESPHome.esphome-vscode`
- Validator: `esphome.validator = local`
- Python interpreter: `/usr/local/bin/python`
- Schema/cache directory: `${HOME}/.esphome-language-server`
- ESPHome YAML file associations:
  - `devices/**/*.yaml`
  - `packages/**/*.yaml`
  - `tests/**/*.yaml`

If validation, hover, or completion does not start after rebuilding the
container:

1. Reload the VSCode window.
2. Confirm the opened YAML file language mode is `ESPHome`.
3. In the integrated terminal, run:

   ```bash
   which esphome
   esphome version
   python -c "import esphome; print(esphome.__version__)"
   ```

4. If ESPHome is missing, rerun:

   ```bash
   bash .devcontainer/install-esphome.sh
   ```

Completion and hover schemas are fetched from `schema.esphome.io`, so those
features also require outbound network access from the devcontainer.

If the output panel shows an error like:

```text
EPERM: operation not permitted, mkdir '<home>/.esphome-language-server'
```

create the directory manually or rerun `.devcontainer/install-esphome.sh`.

If hover/completion still fail after the directory exists, verify the container
can download:

```bash
python - <<'PY'
from urllib.request import urlopen
print(urlopen("https://schema.esphome.io/2026.4.5/schema.zip", timeout=20).status)
PY
```
