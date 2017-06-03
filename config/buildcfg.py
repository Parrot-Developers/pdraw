import sys, os
import dragon

import apps_tools.android as android
import apps_tools.ios as ios

android_pdraw_dir = os.path.join(dragon.WORKSPACE_DIR,
    "packages", "pdraw")
android_jni_dir = os.path.join(android_pdraw_dir,
    "libpdraw", "android", "jni")
android_app_dir = os.path.join(android_pdraw_dir,
    "apps", "pdraw_android")

#===============================================================================
# Android
#===============================================================================

if dragon.VARIANT == "android":
    android_abis = ["armeabi", "armeabi-v7a", "arm64-v8a",
                    "mips",
                    "x86"]

    android.add_task_build_common(android_abis)

    android.add_ndk_build_task(
        name="build-jni",
        desc="Build native libs & jni",
        subtasks=["build-common"],
        calldir=android_jni_dir,
        module="libpdraw",
        abis=android_abis,
        extra_args=["PACKAGES_DIR={}".format(os.path.join(dragon.WORKSPACE_DIR,
                                                          "packages"))]
    )

    android.add_ndk_build_task(
        name="clean-jni",
        desc="Clean native libs & jni",
        calldir=android_jni_dir,
        module="libpdraw",
        abis=android_abis,
        extra_args=["PACKAGES_DIR={}".format(os.path.join(dragon.WORKSPACE_DIR,
                                                          "packages")),
                    "clean"],
        ignore_failure=True
    )

    android.add_gradle_task(
        name="build-app",
        desc="Build the PDrAW Android app in debug",
        subtasks=["build-jni"],
        calldir=android_app_dir,
        target="assembleDebug"
    )

    android.add_gradle_task(
        name="clean-app",
        desc="Clean the PDrAW Android app",
        subtasks=["clean-jni"],
        calldir=android_app_dir,
        target="clean"
    )

    dragon.add_meta_task(
        name="build",
        desc="Build libs & app",
        subtasks=["build-app"]
    )

    dragon.add_meta_task(
        name="clean",
        desc="Clean libs & app",
        subtasks=["clean-app"]
    )
