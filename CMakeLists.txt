# SPDX-License-Identifier: GPL-3.0-only
cmake_minimum_required(VERSION 3.16)

# The board support package comes from FoloToy's ai-passport repository
# (MIT), checked out as a submodule; point PASSPORT_DIR elsewhere to use
# another checkout.
set(PASSPORT_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/ai-passport" CACHE PATH "FoloToy ai-passport checkout")
if(NOT EXISTS "${PASSPORT_DIR}/components/bsp/CMakeLists.txt")
    message(FATAL_ERROR "BSP not found in ${PASSPORT_DIR}: run `git submodule update --init`")
endif()
set(EXTRA_COMPONENT_DIRS "${PASSPORT_DIR}/components/bsp")

# keep in step with main/version.h
set(PROJECT_VER "26.11(3)")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(apricot-sstv-passport)
