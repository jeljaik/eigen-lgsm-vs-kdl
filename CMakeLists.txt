# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8.11)
project(kdlVsLgsm CXX)

# Add cmake modules for Eigen and the other libraries
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# We want to use the local fork of Eigen for testing against different versions
set(Eigen_ROOT_DIR ${PROJECT_SOURCE_DIR}/libraries/eigen)
message("Local Eigen_ROOT_DIR is: " ${Eigen_ROOT_DIR})

# Find Eigen, Eigen-LGSM, Google Test, Orocos KDL
find_package(Eigen REQUIRED)
if (Eigen_FOUND)
    message( "-- Using local Eigen")
    message( "-- Eigen include dir: " ${Eigen_INCLUDE_DIRS})
    message( "-- Eigen version:  " ${Eigen_VERSION})
endif()
#  Eigen_FOUND - system has eigen lib with correct version
#  Eigen_INCLUDE_DIR - the eigen include directory
#  Eigen_VERSION - eigen version

include_directories(${Eigen_INCLUDE_DIRS}
                     install/include)

add_executable(kdlVsLgsm src/main.cpp)

# Linking against gtest libraries. Too lazy to write a proper FindGTest
target_link_libraries(kdlVsLgsm ${CMAKE_SOURCE_DIR}/install/lib/libgtest.a
                                ${CMAKE_SOURCE_DIR}/install/lib/libgtest_main.a
                            ${CMAKE_SOURCE_DIR}/install/lib/liborocos-kdl.dylib)
