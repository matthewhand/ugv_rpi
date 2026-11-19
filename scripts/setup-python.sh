#!/usr/bin/env bash
set -euo pipefail

project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
state="$project/.ugv-python"
environment="$project/ugv-env"
requirements="$project/requirements.txt"
release=20261110
version=3.11.18
pip_options=()
if [[ $# == 2 && $1 == --index-url ]]; then
    pip_options=(--index-url "$2")
elif [[ $# != 0 ]]; then
    printf 'Usage: %s [--index-url URL]\n' "$0" >&2
    exit 2
fi

mkdir -p -- "$state"
exec 9>"$state/setup.lock"
flock 9
download=
candidate=
next_link="$state/next-env"
backup="$state/previous-ugv-env"
link_target=
restore_legacy_environment() {
    if [[ ! -e $environment && ! -L $environment && -d $backup ]]; then
        mv -T -- "$backup" "$environment"
    fi
}
cleanup() {
    restore_legacy_environment
    [[ -z $download ]] || rm -rf -- "$download"
    if [[ -n $candidate && ( -z $link_target || $(readlink -- "$environment" || true) != "$link_target" ) ]]; then
        rm -rf -- "$candidate"
    fi
    rm -f -- "$next_link"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Recover an interrupted switch before starting to build another environment.
restore_legacy_environment
rm -f -- "$next_link"

compatible_python() {
    "$1" -c 'import sys; sys.exit(not (sys.implementation.name == "cpython" and (3, 11, 5) <= sys.version_info[:3] < (3, 12)))' 2>/dev/null
}

if compatible_python /usr/bin/python3; then
    python=/usr/bin/python3
else
    case $(dpkg --print-architecture) in
        arm64)
            target=aarch64-unknown-linux-gnu
            checksum=7e71f26a003949b57757e287f4f4b6689da07e7a229f5bbee7919c19ed77155d
            ;;
        armhf)
            target=armv7-unknown-linux-gnueabihf
            checksum=847d1ceffb1c0ce7a48fd939da88446fb875395266ebc2ea99974f7d6a150102
            ;;
        amd64)
            target=x86_64-unknown-linux-gnu
            checksum=3288c54cea3db350052019b3f8b558dd8a3da8afa6fda573f5fd2bcb37635b4a
            ;;
        *)
            echo 'Use CPython 3.11.5 or newer on this architecture.' >&2
            exit 1
            ;;
    esac
    runtime="$state/cpython-$version-$release-$target"
    python="$runtime/bin/python3"
    if ! compatible_python "$python"; then
        download=$(mktemp -d -- "$state/download.XXXXXXXX")
        asset="cpython-$version+$release-$target-install_only_stripped.tar.gz"
        curl --fail --show-error --silent --location --retry 3 \
            "https://github.com/astral-sh/python-build-standalone/releases/download/$release/$asset" \
            --output "$download/python.tar.gz"
        printf '%s  %s\n' "$checksum" "$download/python.tar.gz" | sha256sum --check --status
        tar -xzf "$download/python.tar.gz" -C "$download"
        compatible_python "$download/python/bin/python3"
        mv -T -- "$download/python" "$runtime"
    fi
fi

fingerprint=$(printf '%s\n%s\n' "$(sha256sum < "$requirements")" "$("$python" -c 'import sys; print(sys.executable, sys.version)')" | sha256sum)
if compatible_python "$environment/bin/python" &&
    [[ -f "$environment/.rover-requirements" ]] &&
    [[ $(cat "$environment/.rover-requirements") == "$fingerprint" ]]; then
    echo 'Rover Python environment is already up to date.'
    exit 0
fi

candidate=$(mktemp -d -- "$state/venv.XXXXXXXX")
"$python" -m venv --system-site-packages "$candidate"
if [[ $python != /usr/bin/python3 && -d /usr/lib/python3/dist-packages ]] &&
    /usr/bin/python3 -c 'import sys; sys.exit(sys.version_info[:2] != (3, 11))' 2>/dev/null; then
    # Standalone CPython does not search Debian's apt-managed packages.
    site_packages=$("$candidate/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')
    printf '%s\n' /usr/lib/python3/dist-packages > "$site_packages/rover-bookworm.pth"
    if /usr/bin/python3 -c 'from importlib.util import find_spec; import sys; sys.exit(find_spec("libcamera") is None)'; then
        "$candidate/bin/python" -c 'import libcamera'
    fi
fi
"$candidate/bin/python" -m pip install "${pip_options[@]}" -r "$requirements"
printf '%s\n' "$fingerprint" > "$candidate/.rover-requirements"

link_target=".ugv-python/$(basename -- "$candidate")"
ln -s -- "$link_target" "$next_link"
if [[ -e $environment && ! -L $environment ]]; then
    if [[ ! -d $environment || -e $backup || -L $backup ]]; then
        printf 'Cannot preserve existing environment at %s. No changes were made to it.\n' "$environment" >&2
        exit 1
    fi
    mv -T -- "$environment" "$backup"
fi
mv -Tf -- "$next_link" "$environment"
candidate=
printf 'Rover Python environment ready at %s\n' "$environment"
