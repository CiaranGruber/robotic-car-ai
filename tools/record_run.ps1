# Record one system-ID test on the car, copy it back, and analyse it.
#
# Usage (from the ai4r_policy folder, in PowerShell):
#   powershell -ExecutionPolicy Bypass -File tools\record_run.ps1 -Car <CAR_IP> -Name cal_speed_8
#
# 1. Starts ros2 bag record on the car (one ssh session, no extra window).
# 2. You run the test in Foxglove as usual.
# 3. Press Enter in this window to stop recording cleanly.
# 4. The bag is copied into .\bags\ and tools\sysid_analyze.py prints the result.
# A timestamp is added to the name, so reruns never collide.
param(
    [Parameter(Mandatory = $true)][string]$Car,
    [string]$Name = "run",
    [string]$User = "ai4r"
)
$ErrorActionPreference = "Stop"
$bag = "{0}_{1}" -f $Name, (Get-Date -Format "yyyyMMdd_HHmmss")
$topics = @(
    "/car/drive_and_steer_set_point_normalized", "/car/drive_and_steer_applied_normalized",
    "/car/wheel_speed_m_per_sec", "/car/encoder_period_seconds", "/car/debug1",
    "/car/debug2", "/car/imu/data", "/rosout") -join " "

# The recorder runs in the background on the car; 'read' waits for Enter,
# then SIGINT lets rosbag2 close the file properly. The remote script is
# wrapped in single quotes and contains no double quotes, because Windows
# PowerShell 5.1 mangles embedded double quotes passed to ssh.exe.
# 'set -m' keeps job control on, otherwise bash makes the background
# recorder ignore SIGINT and the bag is never closed.
# '</dev/null': Jazzy's recorder reads the terminal for keyboard controls,
# and a background job that reads the tty is stopped (SIGTTIN) before it
# records anything (car 20, 5 Oct).
$script = "set -m; source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; " +
          "ros2 bag record -s mcap -o ~/$bag --topics $topics </dev/null >/tmp/$bag.log 2>&1 & p=`$!; " +
          "sleep 3; echo; echo === Recording ~/$bag - run the test in Foxglove, then press Enter to stop ===; " +
          "read _; kill -INT `$p; wait `$p; echo === Saved ===; tail -n 3 /tmp/$bag.log"
ssh -t "$User@$Car" "bash -ic '$script'"
if ($LASTEXITCODE -ne 0) { throw "Recording over ssh failed (exit $LASTEXITCODE)" }

New-Item -ItemType Directory -Force "bags" | Out-Null
scp -r "${User}@${Car}:$bag" "bags/"
if ($LASTEXITCODE -ne 0) { throw "Copying ~/$bag back failed" }

$mcap = Get-ChildItem "bags/$bag" -Filter *.mcap | Select-Object -First 1
if (-not $mcap) { throw "No .mcap file in bags/$bag" }
Write-Host ">>> Copied to bags/$bag"
py tools/sysid_analyze.py $mcap.FullName
