import pathlib
import platform
import subprocess
import urllib.request
import tempfile
import json
import sys


# ChatGTP generated most of this
VS_URL = "https://aka.ms/vs/release/vs_BuildTools.exe"

VSWHERE_PATHS = [
    "C:/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe",
    "C:/Program Files/Microsoft Visual Studio/Installer/vswhere.exe"
]

MSVC_VERSION_MINIMUM = "17.14"

def find_vswhere():
    for path in VSWHERE_PATHS:
        if pathlib.Path(path).is_file():
            return path
    return None


def is_msvc_installed():
    """
    Use vswhere to check for MSVC Build Tools installation.
    """
    vswhere = find_vswhere()
    if not vswhere:
        print("vswhere.exe not found — assuming MSVC is NOT installed.")
        return False

    try:
        # Query installations that include the VC Tools workload
        print(vswhere)
        cmd = [
            vswhere,
            "-products", "*",
            "-requires", "Microsoft.VisualStudio.Workload.VCTools",
            "-format", "json"
        ]

        result = subprocess.run(cmd, capture_output=True)
        if result.returncode != 0:
            return False

        json_str = result.stdout.decode("cp1252", errors="replace")

        installations = json.loads(json_str)


        good_install = False
        if installations:
            print(f"MSVC Installed versions: {[installation['installationVersion'] for installation in installations]}")
            for installation in installations:
                version = installation["installationVersion"].split(".")
                required_version = MSVC_VERSION_MINIMUM.split(".")
                if int(version[0]) > int(required_version[0]):
                    good_install = True
                elif int(version[0]) == int(required_version[0]) and int(version[1]) >= int(required_version[1]):
                    good_install = True
            if not good_install:
                print("MSVC found, but only old versions!")
            return good_install

        print("MSVC missing, installing...")
        return False

    except Exception as e:
        print(f"Error while checking MSVC installation: {e}")
        return False


def download_file(url, dest_path):
    print(f"Downloading installer from:\n{url}\n")
    headers = {
        "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)"
    }
    req = urllib.request.Request(url, headers=headers)
    with urllib.request.urlopen(req) as response, open(dest_path, "wb") as out:
        data = response.read()
        out.write(data)
    print(f"Downloaded to: {dest_path}\n")


def install_msvc(installer_path):
    # Returns True if a reinstall is required, otherwise False
    print("Starting silent MSVC Build Tools installation...\n")

    components = [
        "--add", "Microsoft.VisualStudio.Workload.VCTools",
        "--includeRecommended"
    ]

    cmd = [
        installer_path,
        "--quiet",
        "--wait",
        "--norestart",
        "--nocache",
    ] + components

    print("Running command:\n" + " ".join(cmd) + "\n")

    try:
        subprocess.run(cmd, check=True, stdout=sys.stdout, stderr=sys.stderr)
    except subprocess.CalledProcessError as e:
        if e.returncode == 3010:
            print("Restart required to finish installation! Please restart the machine and then run the script again!")
            return True
        else:
            raise
    print("MSVC Build Tools installation completed successfully.")
    return False


def msvc():
    # Returns True if we need to restart for MSVC
    if is_msvc_installed():
        print("MSVC is already installed — skipping download and installation.")
        return False

    print("MSVC not detected. Proceeding with installation...\n")

    with tempfile.TemporaryDirectory() as tempdir:
        installer_path = str(pathlib.Path(tempdir).joinpath("vs_buildtools.exe"))
        download_file(VS_URL, installer_path)
        need_restart = install_msvc(installer_path)
        return need_restart

def main():
    if platform.system() == "Windows":
        need_restart = msvc()
        if not need_restart:
            print("All done!")
    else:
        print("Not on Windows, this step is not necessary!")


if __name__ == "__main__":
    main()
