#!/usr/bin/env python3
import argparse
import os
import subprocess
import shutil
runcmd = subprocess.run

repo_root = os.path.dirname(os.path.abspath(__file__))
container_name = 'sentry-dev'

docker_run_args: list[str] = [
    "-it",
    "--rm",
    "-e",  "COLORTERM",
    "-e",  "TERM=xterm-256color",
    "-v",  f"{repo_root}:/ws:Z",
    "-w",  "/ws",
]
colcon_args: list[str] = [
    "--symlink-install",
    "--event-handlers",
    "console_cohesion+",
    "--mixin",
    "compile-commands",
    "clang",
    "ninja",
]

def in_container() -> bool:
    return not os.getenv("IN_CONTAINER") is None

def build(args) -> None:
    if in_container():
        print("This repository action is meant for outside container use")
    else:
        cmd = [args.docker_cmd, "build"]
        if args.docker_cmd == "docker":
            cmd.extend(["--progress=plain"])
        cmd.extend(['-t', args.image_name, '-f', 'Dockerfile', '.'])
        runcmd(cmd)


def shell(args) -> None:
    check = runcmd([args.docker_cmd, "container", "inspect", args.container_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if check.returncode == 0:
        print("Entering old container..")
        runcmd([args.docker_cmd, "exec", "-it", args.container_name, "bash"])
    else:
        build(args)
        cmd = [args.docker_cmd, "run", *docker_run_args, f"--name={args.container_name}"]
        if args.display:
            cmd.extend([
                       '--privileged', '--net=host',
                       '--ipc=host', '-e=DISPLAY',
                       '-v=/tmp/.X11-unix:/tmp/.X11-unix:rw'])
        cmd.extend([args.image_name, 'bash'])
        runcmd(cmd)


def compile(args) -> None:
    if not in_container():
        print("compile is an in-container command, please enter a shell using ./repo.py shell")
    else:
        cmd = ["colcon", "build", *colcon_args]
        if args.package is None:
            runcmd(cmd)
        else:
            runcmd([*cmd, "--packages-up-to", args.package])

def find_default_docker() -> str:
    for cmd in ["podman", "docker"]:
        if shutil.which(cmd) is not None:
            return cmd
    else:
        return "docker"


def add_docker_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--docker-cmd", type=str, default=find_default_docker(), help="Docker (or Podman) binary")
    parser.add_argument("--image-name", type=str, default="northstar/sentry:arc2", help="Container image name")


def main() -> None:
    parser = argparse.ArgumentParser(description="Repo actions script")
    subparsers = parser.add_subparsers(dest="command", required=True, help="subcommands")

    build_parser = subparsers.add_parser("build", help="Build the container image")
    add_docker_args(build_parser)

    shell_parser = subparsers.add_parser("shell", help="Enter a shell inside the container")
    add_docker_args(shell_parser)
    shell_parser.add_argument("--container-name", type=str, default=container_name, help="Container name")
    shell_parser.add_argument("--display", action="store_true", help="Forward display")

    compile_parser = subparsers.add_parser("compile", help="Compile the ROS 2 workspace")
    compile_parser.add_argument("--package", type=str, default=None, help="Specify a package to build")
    compile_parser.add_argument("--build-type", type=str, default="Debug", choices=["Debug", "Release", "RelWithDebInfo"], help="Specify colcon build type")

    args = parser.parse_args()
    globals()[args.command](args)


if __name__ == "__main__":
    main()
