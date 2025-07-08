# Changelog

## [2.2.1] - 2025-07-08

### <!-- 1 -->🐛 Bug Fixes

- Fix grid coordinate dimension assignment

## [2.2.0] - 2025-07-02

### <!-- 4 -->⚡ Performance

- Refactor grid calculations and optimize access methods
- Optimize grid coordinate and point calculations

## [2.1.0] - 2025-07-02

### <!-- 0 -->⛰️  Features

- Feature: Implement unified spatial indexing API

## [2.0.4] - 2025-06-28

### <!-- 0 -->⛰️  Features

- Improve geospatial data processing efficiency and reliability

### <!-- 1 -->🐛 Bug Fixes

- Improve R-tree node splitting robustness

## [2.0.3] - 2025-06-17

### <!-- 2 -->🚜 Refactor

- Refactor grid constructors datum argument usage

## [2.0.2] - 2025-06-16

### <!-- 0 -->⛰️  Features

- Add CRS enum and WGS conversions

## [2.0.1] - 2025-06-15

### <!-- 0 -->⛰️  Features

- Add Point constructor for 2D coordinates

## [2.0.0] - 2025-06-15

### <!-- 0 -->⛰️  Features

- Implement new template coordinate conversion builder
- Implement R-tree spatial index and update core structure

### <!-- 2 -->🚜 Refactor

- Refactor geographic coordinate system types

## [1.3.3] - 2025-06-12

### <!-- 0 -->⛰️  Features

- Add intelligent polygon partitioning

### Build

- Log build server errors in quickfix
- Add makefile targets for help and clean

## [1.3.1] - 2025-06-11

### <!-- 5 -->🎨 Styling

- Improve C++ code style and organization

### <!-- 6 -->🧪 Testing

- Refactor CMake build process and testing
- Refactor and add unit testing infrastructure

### Build

- Replace `run.sh` with `make`
- Update project name handling in build scripts

## [1.3.0] - 2025-06-10

### <!-- 0 -->⛰️  Features

- Add comprehensive polygon partition and triangulation algorithms
- Add extended coordinate system and geometric types
- Add Grid constructor from Polygon
- Add getter for path and polygon points
- Add enum for coordinate reference systems
- Allow linkage via concord::concord
- Add trig utility functions, polygon and point methods
- Add coordinate transformation utilities
- Add geometry `is_set()` functions
- Refactor Point and Polygon coordinate handling
- Add support for oriented bounding boxes
- Add utility functions for polygon analysis
- Add pose and rotation to Grid constructor
- Convert corners to WGS84 and add optional datum
- Refactor `Grid` methods and add `corners`
- Initialize Point struct with ENU coordinates
- Add datum support to as_polygon function
- Add constructor to Pose struct
- Refactor pose bounding box representation
- Flatten grid points into a 3D vector
- Represent circle as polygon
- Add option to center grid coordinate system
- Introduce contains method to geometry types
- Add polygon constructor from rectangle
- Apply optional point inflation to rectangles and points
- Add polygon creation from ENU, WGS, and rect
- Templatize Grid class and add point containment
- Feat: Enable ENU and WGS coordinate conversion
- Add constructor to `Point` with ENU and Datum
- Make Datum optional in the Point struct
- Feat: Add Datum field to Point structure
- Refactor geometry types to include Datum
- Add Point constructor for ENU and WGS
- Add basic geometric primitives
- Refactor datum and geometry conversions
- Add Square type and raise recordings
- Introduce base Rectangle type support
- Add basic geometric shape structs
- Add WGS and ENU coordinate conversions
- Add structs for common coordinate systems
- Switch to internal Concord API for library declarations
- Enforce Standards for Consistent Representation of Geometric Constants
- Switch to concord fork for dependencies and API
- Generalize vendor API and minimize external dependencies.
- Implement core functionality for converting geographic coordinates
- Init

### <!-- 1 -->🐛 Bug Fixes

- Improve precision and testing for coordinate conversions
- Fix corner order in `get_corners`

### <!-- 2 -->🚜 Refactor

- Make `flatten_points` a const method
- Refactor header inclusion system
- Refactor geometry types and algorithms
- Refactor coordinate utilities and types
- Move core types to individual headers
- Refactor: Clean up and improve grid element handling
- Construct Grid object from Polygon
- Use minimum area oriented bounding box calc
- Refactor polygon bounding box calculations
- Cast double values to float explicitly
- Refactor Point and Pose structs
- Refactor Grid initialization with default constructors
- Improve memory safety and performance with `std::span`
- Initialize geometric types in constructors
- Remove redundant Point and coordinate constructors
- Introduce and use Datum struct for coordinates
- "Simplify CMake build and directory structure"

### <!-- 3 -->📚 Documentation

- Introduce the long-term Concord C++ library plan
- Improve documentation and refactor IO utilities
- Remove redundant project change log
- Update README to Reflect Changes and corrections
- Introduce revamped Concord++ library with improved documentation and reduced dependencies

### <!-- 4 -->⚡ Performance

- Add inline keyword to spatial types and algos
- Switch to internal API for ALIAS

### <!-- 5 -->🎨 Styling

- Remove redundant explicit keyword from ctors
- "Refactor internal API and improve style consistency"

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Remove spdlog and add library alias
- Refactor cmake build and examples
- "get_summary_prefix", "parameters": {"prefix": "build"}}: update CMake version and library dependencies
- "Update CMake configuration for dependencies and build on multiple platforms"
- Logo colors
- Logo colors

### Build

- Rename BUILD_EXAMPLES cmake option
- Rename EXAMPLE support CMake option
- Add Makefile aliases for building and testing
- Remove unused CMakeLists.txt_2 file
- Add spdlog dependency when build examples
- Refactor installer to remove example includes
- Improve development environment setup and build system
- Add Nix/Devbox for development environment setup
- "Improve build system and library export with package-config support"

## [1.2.0] - 2025-05-31

### <!-- 0 -->⛰️  Features

- Add extended coordinate system and geometric types
- Add Grid constructor from Polygon
- Add getter for path and polygon points
- Add enum for coordinate reference systems
- Allow linkage via concord::concord
- Add trig utility functions, polygon and point methods
- Add coordinate transformation utilities
- Add geometry `is_set()` functions
- Refactor Point and Polygon coordinate handling
- Add support for oriented bounding boxes
- Add utility functions for polygon analysis
- Add pose and rotation to Grid constructor
- Convert corners to WGS84 and add optional datum
- Refactor `Grid` methods and add `corners`
- Initialize Point struct with ENU coordinates
- Add datum support to as_polygon function
- Add constructor to Pose struct
- Refactor pose bounding box representation
- Flatten grid points into a 3D vector
- Represent circle as polygon
- Add option to center grid coordinate system
- Introduce contains method to geometry types
- Add polygon constructor from rectangle
- Apply optional point inflation to rectangles and points
- Add polygon creation from ENU, WGS, and rect
- Templatize Grid class and add point containment
- Feat: Enable ENU and WGS coordinate conversion
- Add constructor to `Point` with ENU and Datum
- Make Datum optional in the Point struct
- Feat: Add Datum field to Point structure
- Refactor geometry types to include Datum
- Add Point constructor for ENU and WGS
- Add basic geometric primitives
- Refactor datum and geometry conversions
- Add Square type and raise recordings
- Introduce base Rectangle type support
- Add basic geometric shape structs
- Add WGS and ENU coordinate conversions
- Add structs for common coordinate systems
- Switch to internal Concord API for library declarations
- Enforce Standards for Consistent Representation of Geometric Constants
- Switch to concord fork for dependencies and API
- Generalize vendor API and minimize external dependencies.
- Implement core functionality for converting geographic coordinates
- Init

### <!-- 1 -->🐛 Bug Fixes

- Fix corner order in `get_corners`

### <!-- 2 -->🚜 Refactor

- Refactor: Clean up and improve grid element handling
- Construct Grid object from Polygon
- Use minimum area oriented bounding box calc
- Refactor polygon bounding box calculations
- Cast double values to float explicitly
- Refactor Point and Pose structs
- Refactor Grid initialization with default constructors
- Improve memory safety and performance with `std::span`
- Initialize geometric types in constructors
- Remove redundant Point and coordinate constructors
- Introduce and use Datum struct for coordinates
- "Simplify CMake build and directory structure"

### <!-- 3 -->📚 Documentation

- Improve documentation and refactor IO utilities
- Remove redundant project change log
- Update README to Reflect Changes and corrections
- Introduce revamped Concord++ library with improved documentation and reduced dependencies

### <!-- 4 -->⚡ Performance

- Add inline keyword to spatial types and algos
- Switch to internal API for ALIAS

### <!-- 5 -->🎨 Styling

- Remove redundant explicit keyword from ctors
- "Refactor internal API and improve style consistency"

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Remove spdlog and add library alias
- Refactor cmake build and examples
- "get_summary_prefix", "parameters": {"prefix": "build"}}: update CMake version and library dependencies
- "Update CMake configuration for dependencies and build on multiple platforms"
- Logo colors
- Logo colors

### Build

- Rename BUILD_EXAMPLES cmake option
- Rename EXAMPLE support CMake option
- Add Makefile aliases for building and testing
- Remove unused CMakeLists.txt_2 file
- Add spdlog dependency when build examples
- Refactor installer to remove example includes
- Improve development environment setup and build system
- Add Nix/Devbox for development environment setup
- "Improve build system and library export with package-config support"

## [1.1.0] - 2025-05-31

### <!-- 0 -->⛰️  Features

- Add extended coordinate system and geometric types
- Add Grid constructor from Polygon
- Add getter for path and polygon points
- Add enum for coordinate reference systems
- Allow linkage via concord::concord
- Add trig utility functions, polygon and point methods
- Add coordinate transformation utilities
- Add geometry `is_set()` functions
- Refactor Point and Polygon coordinate handling
- Add support for oriented bounding boxes
- Add utility functions for polygon analysis
- Add pose and rotation to Grid constructor
- Convert corners to WGS84 and add optional datum
- Refactor `Grid` methods and add `corners`
- Initialize Point struct with ENU coordinates
- Add datum support to as_polygon function
- Add constructor to Pose struct
- Refactor pose bounding box representation
- Flatten grid points into a 3D vector
- Represent circle as polygon
- Add option to center grid coordinate system
- Introduce contains method to geometry types
- Add polygon constructor from rectangle
- Apply optional point inflation to rectangles and points
- Add polygon creation from ENU, WGS, and rect
- Templatize Grid class and add point containment
- Feat: Enable ENU and WGS coordinate conversion
- Add constructor to `Point` with ENU and Datum
- Make Datum optional in the Point struct
- Feat: Add Datum field to Point structure
- Refactor geometry types to include Datum
- Add Point constructor for ENU and WGS
- Add basic geometric primitives
- Refactor datum and geometry conversions
- Add Square type and raise recordings
- Introduce base Rectangle type support
- Add basic geometric shape structs
- Add WGS and ENU coordinate conversions
- Add structs for common coordinate systems
- Switch to internal Concord API for library declarations
- Enforce Standards for Consistent Representation of Geometric Constants
- Switch to concord fork for dependencies and API
- Generalize vendor API and minimize external dependencies.
- Implement core functionality for converting geographic coordinates
- Init

### <!-- 1 -->🐛 Bug Fixes

- Fix corner order in `get_corners`

### <!-- 2 -->🚜 Refactor

- Refactor: Clean up and improve grid element handling
- Construct Grid object from Polygon
- Use minimum area oriented bounding box calc
- Refactor polygon bounding box calculations
- Cast double values to float explicitly
- Refactor Point and Pose structs
- Refactor Grid initialization with default constructors
- Improve memory safety and performance with `std::span`
- Initialize geometric types in constructors
- Remove redundant Point and coordinate constructors
- Introduce and use Datum struct for coordinates
- "Simplify CMake build and directory structure"

### <!-- 3 -->📚 Documentation

- Improve documentation and refactor IO utilities
- Remove redundant project change log
- Update README to Reflect Changes and corrections
- Introduce revamped Concord++ library with improved documentation and reduced dependencies

### <!-- 4 -->⚡ Performance

- Add inline keyword to spatial types and algos
- Switch to internal API for ALIAS

### <!-- 5 -->🎨 Styling

- Remove redundant explicit keyword from ctors
- "Refactor internal API and improve style consistency"

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Remove spdlog and add library alias
- Refactor cmake build and examples
- "get_summary_prefix", "parameters": {"prefix": "build"}}: update CMake version and library dependencies
- "Update CMake configuration for dependencies and build on multiple platforms"
- Logo colors
- Logo colors

### Build

- Rename BUILD_EXAMPLES cmake option
- Rename EXAMPLE support CMake option
- Add Makefile aliases for building and testing
- Remove unused CMakeLists.txt_2 file
- Add spdlog dependency when build examples
- Refactor installer to remove example includes
- Improve development environment setup and build system
- Add Nix/Devbox for development environment setup
- "Improve build system and library export with package-config support"

## [1.0.4] - 2025-05-31

### <!-- 0 -->⛰️  Features

- Add extended coordinate system and geometric types
- Add Grid constructor from Polygon
- Add getter for path and polygon points
- Add enum for coordinate reference systems
- Allow linkage via concord::concord
- Add trig utility functions, polygon and point methods
- Add coordinate transformation utilities
- Add geometry `is_set()` functions
- Refactor Point and Polygon coordinate handling
- Add support for oriented bounding boxes
- Add utility functions for polygon analysis
- Add pose and rotation to Grid constructor
- Convert corners to WGS84 and add optional datum
- Refactor `Grid` methods and add `corners`
- Initialize Point struct with ENU coordinates
- Add datum support to as_polygon function
- Add constructor to Pose struct
- Refactor pose bounding box representation
- Flatten grid points into a 3D vector
- Represent circle as polygon
- Add option to center grid coordinate system
- Introduce contains method to geometry types
- Add polygon constructor from rectangle
- Apply optional point inflation to rectangles and points
- Add polygon creation from ENU, WGS, and rect
- Templatize Grid class and add point containment
- Feat: Enable ENU and WGS coordinate conversion
- Add constructor to `Point` with ENU and Datum
- Make Datum optional in the Point struct
- Feat: Add Datum field to Point structure
- Refactor geometry types to include Datum
- Add Point constructor for ENU and WGS
- Add basic geometric primitives
- Refactor datum and geometry conversions
- Add Square type and raise recordings
- Introduce base Rectangle type support
- Add basic geometric shape structs
- Add WGS and ENU coordinate conversions
- Add structs for common coordinate systems
- Switch to internal Concord API for library declarations
- Enforce Standards for Consistent Representation of Geometric Constants
- Switch to concord fork for dependencies and API
- Generalize vendor API and minimize external dependencies.
- Implement core functionality for converting geographic coordinates
- Init

### <!-- 1 -->🐛 Bug Fixes

- Fix corner order in `get_corners`

### <!-- 2 -->🚜 Refactor

- Refactor: Clean up and improve grid element handling
- Construct Grid object from Polygon
- Use minimum area oriented bounding box calc
- Refactor polygon bounding box calculations
- Cast double values to float explicitly
- Refactor Point and Pose structs
- Refactor Grid initialization with default constructors
- Improve memory safety and performance with `std::span`
- Initialize geometric types in constructors
- Remove redundant Point and coordinate constructors
- Introduce and use Datum struct for coordinates
- "Simplify CMake build and directory structure"

### <!-- 3 -->📚 Documentation

- Improve documentation and refactor IO utilities
- Remove redundant project change log
- Update README to Reflect Changes and corrections
- Introduce revamped Concord++ library with improved documentation and reduced dependencies

### <!-- 4 -->⚡ Performance

- Add inline keyword to spatial types and algos
- Switch to internal API for ALIAS

### <!-- 5 -->🎨 Styling

- Remove redundant explicit keyword from ctors
- "Refactor internal API and improve style consistency"

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Remove spdlog and add library alias
- Refactor cmake build and examples
- "get_summary_prefix", "parameters": {"prefix": "build"}}: update CMake version and library dependencies
- "Update CMake configuration for dependencies and build on multiple platforms"
- Logo colors
- Logo colors

### Build

- Rename BUILD_EXAMPLES cmake option
- Rename EXAMPLE support CMake option
- Add Makefile aliases for building and testing
- Remove unused CMakeLists.txt_2 file
- Add spdlog dependency when build examples
- Refactor installer to remove example includes
- Improve development environment setup and build system
- Add Nix/Devbox for development environment setup
- "Improve build system and library export with package-config support"

## [1.0.3] - 2025-05-31

### <!-- 0 -->⛰️  Features

- Add extended coordinate system and geometric types
- Add Grid constructor from Polygon
- Add getter for path and polygon points
- Add enum for coordinate reference systems
- Allow linkage via concord::concord
- Add trig utility functions, polygon and point methods
- Add coordinate transformation utilities
- Add geometry `is_set()` functions
- Refactor Point and Polygon coordinate handling
- Add support for oriented bounding boxes
- Add utility functions for polygon analysis
- Add pose and rotation to Grid constructor
- Convert corners to WGS84 and add optional datum
- Refactor `Grid` methods and add `corners`
- Initialize Point struct with ENU coordinates
- Add datum support to as_polygon function
- Add constructor to Pose struct
- Refactor pose bounding box representation
- Flatten grid points into a 3D vector
- Represent circle as polygon
- Add option to center grid coordinate system
- Introduce contains method to geometry types
- Add polygon constructor from rectangle
- Apply optional point inflation to rectangles and points
- Add polygon creation from ENU, WGS, and rect
- Templatize Grid class and add point containment
- Feat: Enable ENU and WGS coordinate conversion
- Add constructor to `Point` with ENU and Datum
- Make Datum optional in the Point struct
- Feat: Add Datum field to Point structure
- Refactor geometry types to include Datum
- Add Point constructor for ENU and WGS
- Add basic geometric primitives
- Refactor datum and geometry conversions
- Add Square type and raise recordings
- Introduce base Rectangle type support
- Add basic geometric shape structs
- Add WGS and ENU coordinate conversions
- Add structs for common coordinate systems
- Switch to internal Concord API for library declarations
- Enforce Standards for Consistent Representation of Geometric Constants
- Switch to concord fork for dependencies and API
- Generalize vendor API and minimize external dependencies.
- Implement core functionality for converting geographic coordinates
- Init

### <!-- 1 -->🐛 Bug Fixes

- Fix corner order in `get_corners`

### <!-- 2 -->🚜 Refactor

- Refactor: Clean up and improve grid element handling
- Construct Grid object from Polygon
- Use minimum area oriented bounding box calc
- Refactor polygon bounding box calculations
- Cast double values to float explicitly
- Refactor Point and Pose structs
- Refactor Grid initialization with default constructors
- Improve memory safety and performance with `std::span`
- Initialize geometric types in constructors
- Remove redundant Point and coordinate constructors
- Introduce and use Datum struct for coordinates
- "Simplify CMake build and directory structure"

### <!-- 3 -->📚 Documentation

- Improve documentation and refactor IO utilities
- Remove redundant project change log
- Update README to Reflect Changes and corrections
- Introduce revamped Concord++ library with improved documentation and reduced dependencies

### <!-- 4 -->⚡ Performance

- Add inline keyword to spatial types and algos
- Switch to internal API for ALIAS

### <!-- 5 -->🎨 Styling

- Remove redundant explicit keyword from ctors
- "Refactor internal API and improve style consistency"

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Remove spdlog and add library alias
- Refactor cmake build and examples
- "get_summary_prefix", "parameters": {"prefix": "build"}}: update CMake version and library dependencies
- "Update CMake configuration for dependencies and build on multiple platforms"
- Logo colors
- Logo colors

### Build

- Rename BUILD_EXAMPLES cmake option
- Rename EXAMPLE support CMake option
- Add Makefile aliases for building and testing
- Remove unused CMakeLists.txt_2 file
- Add spdlog dependency when build examples
- Refactor installer to remove example includes
- Improve development environment setup and build system
- Add Nix/Devbox for development environment setup
- "Improve build system and library export with package-config support"

## [1.0.2] - 2025-05-31

### <!-- 0 -->⛰️  Features

- Add extended coordinate system and geometric types
- Add Grid constructor from Polygon
- Add getter for path and polygon points
- Add enum for coordinate reference systems
- Allow linkage via concord::concord
- Add trig utility functions, polygon and point methods
- Add coordinate transformation utilities
- Add geometry `is_set()` functions
- Refactor Point and Polygon coordinate handling
- Add support for oriented bounding boxes
- Add utility functions for polygon analysis
- Add pose and rotation to Grid constructor
- Convert corners to WGS84 and add optional datum
- Refactor `Grid` methods and add `corners`
- Initialize Point struct with ENU coordinates
- Add datum support to as_polygon function
- Add constructor to Pose struct
- Refactor pose bounding box representation
- Flatten grid points into a 3D vector
- Represent circle as polygon
- Add option to center grid coordinate system
- Introduce contains method to geometry types
- Add polygon constructor from rectangle
- Apply optional point inflation to rectangles and points
- Add polygon creation from ENU, WGS, and rect
- Templatize Grid class and add point containment
- Feat: Enable ENU and WGS coordinate conversion
- Add constructor to `Point` with ENU and Datum
- Make Datum optional in the Point struct
- Feat: Add Datum field to Point structure
- Refactor geometry types to include Datum
- Add Point constructor for ENU and WGS
- Add basic geometric primitives
- Refactor datum and geometry conversions
- Add Square type and raise recordings
- Introduce base Rectangle type support
- Add basic geometric shape structs
- Add WGS and ENU coordinate conversions
- Add structs for common coordinate systems
- Switch to internal Concord API for library declarations
- Enforce Standards for Consistent Representation of Geometric Constants
- Switch to concord fork for dependencies and API
- Generalize vendor API and minimize external dependencies.
- Implement core functionality for converting geographic coordinates
- Init

### <!-- 1 -->🐛 Bug Fixes

- Fix corner order in `get_corners`

### <!-- 2 -->🚜 Refactor

- Refactor: Clean up and improve grid element handling
- Construct Grid object from Polygon
- Use minimum area oriented bounding box calc
- Refactor polygon bounding box calculations
- Cast double values to float explicitly
- Refactor Point and Pose structs
- Refactor Grid initialization with default constructors
- Improve memory safety and performance with `std::span`
- Initialize geometric types in constructors
- Remove redundant Point and coordinate constructors
- Introduce and use Datum struct for coordinates
- "Simplify CMake build and directory structure"

### <!-- 3 -->📚 Documentation

- Improve documentation and refactor IO utilities
- Remove redundant project change log
- Update README to Reflect Changes and corrections
- Introduce revamped Concord++ library with improved documentation and reduced dependencies

### <!-- 4 -->⚡ Performance

- Add inline keyword to spatial types and algos
- Switch to internal API for ALIAS

### <!-- 5 -->🎨 Styling

- Remove redundant explicit keyword from ctors
- "Refactor internal API and improve style consistency"

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Remove spdlog and add library alias
- Refactor cmake build and examples
- "get_summary_prefix", "parameters": {"prefix": "build"}}: update CMake version and library dependencies
- "Update CMake configuration for dependencies and build on multiple platforms"
- Logo colors
- Logo colors

### Build

- Rename BUILD_EXAMPLES cmake option
- Rename EXAMPLE support CMake option
- Add Makefile aliases for building and testing
- Remove unused CMakeLists.txt_2 file
- Add spdlog dependency when build examples
- Refactor installer to remove example includes
- Improve development environment setup and build system
- Add Nix/Devbox for development environment setup
- "Improve build system and library export with package-config support"

## [1.0.1] - 2025-05-30

### <!-- 0 -->⛰️  Features

- Add extended coordinate system and geometric types
- Add Grid constructor from Polygon
- Add getter for path and polygon points
- Add enum for coordinate reference systems
- Allow linkage via concord::concord
- Add trig utility functions, polygon and point methods
- Add coordinate transformation utilities
- Add geometry `is_set()` functions
- Refactor Point and Polygon coordinate handling
- Add support for oriented bounding boxes
- Add utility functions for polygon analysis
- Add pose and rotation to Grid constructor
- Convert corners to WGS84 and add optional datum
- Refactor `Grid` methods and add `corners`
- Initialize Point struct with ENU coordinates
- Add datum support to as_polygon function
- Add constructor to Pose struct
- Refactor pose bounding box representation
- Flatten grid points into a 3D vector
- Represent circle as polygon
- Add option to center grid coordinate system
- Introduce contains method to geometry types
- Add polygon constructor from rectangle
- Apply optional point inflation to rectangles and points
- Add polygon creation from ENU, WGS, and rect
- Templatize Grid class and add point containment
- Feat: Enable ENU and WGS coordinate conversion
- Add constructor to `Point` with ENU and Datum
- Make Datum optional in the Point struct
- Feat: Add Datum field to Point structure
- Refactor geometry types to include Datum
- Add Point constructor for ENU and WGS
- Add basic geometric primitives
- Refactor datum and geometry conversions
- Add Square type and raise recordings
- Introduce base Rectangle type support
- Add basic geometric shape structs
- Add WGS and ENU coordinate conversions
- Add structs for common coordinate systems
- Switch to internal Concord API for library declarations
- Enforce Standards for Consistent Representation of Geometric Constants
- Switch to concord fork for dependencies and API
- Generalize vendor API and minimize external dependencies.
- Implement core functionality for converting geographic coordinates
- Init

### <!-- 1 -->🐛 Bug Fixes

- Fix corner order in `get_corners`

### <!-- 2 -->🚜 Refactor

- Refactor: Clean up and improve grid element handling
- Construct Grid object from Polygon
- Use minimum area oriented bounding box calc
- Refactor polygon bounding box calculations
- Cast double values to float explicitly
- Refactor Point and Pose structs
- Refactor Grid initialization with default constructors
- Improve memory safety and performance with `std::span`
- Initialize geometric types in constructors
- Remove redundant Point and coordinate constructors
- Introduce and use Datum struct for coordinates
- "Simplify CMake build and directory structure"

### <!-- 3 -->📚 Documentation

- Improve documentation and refactor IO utilities
- Remove redundant project change log
- Update README to Reflect Changes and corrections
- Introduce revamped Concord++ library with improved documentation and reduced dependencies

### <!-- 4 -->⚡ Performance

- Switch to internal API for ALIAS

### <!-- 5 -->🎨 Styling

- Remove redundant explicit keyword from ctors
- "Refactor internal API and improve style consistency"

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Remove spdlog and add library alias
- Refactor cmake build and examples
- "get_summary_prefix", "parameters": {"prefix": "build"}}: update CMake version and library dependencies
- "Update CMake configuration for dependencies and build on multiple platforms"
- Logo colors
- Logo colors

### Build

- Remove unused CMakeLists.txt_2 file
- Add spdlog dependency when build examples
- Refactor installer to remove example includes
- Improve development environment setup and build system
- Add Nix/Devbox for development environment setup
- "Improve build system and library export with package-config support"

## [1.0.0] - 2025-05-30

### <!-- 0 -->⛰️  Features

- Add extended coordinate system and geometric types
- Add Grid constructor from Polygon
- Add getter for path and polygon points
- Add enum for coordinate reference systems
- Allow linkage via concord::concord
- Add trig utility functions, polygon and point methods
- Add coordinate transformation utilities
- Add geometry `is_set()` functions
- Refactor Point and Polygon coordinate handling
- Add support for oriented bounding boxes
- Add utility functions for polygon analysis
- Add pose and rotation to Grid constructor
- Convert corners to WGS84 and add optional datum
- Refactor `Grid` methods and add `corners`
- Initialize Point struct with ENU coordinates
- Add datum support to as_polygon function
- Add constructor to Pose struct
- Refactor pose bounding box representation
- Flatten grid points into a 3D vector
- Represent circle as polygon
- Add option to center grid coordinate system
- Introduce contains method to geometry types
- Add polygon constructor from rectangle
- Apply optional point inflation to rectangles and points
- Add polygon creation from ENU, WGS, and rect
- Templatize Grid class and add point containment
- Feat: Enable ENU and WGS coordinate conversion
- Add constructor to `Point` with ENU and Datum
- Make Datum optional in the Point struct
- Feat: Add Datum field to Point structure
- Refactor geometry types to include Datum
- Add Point constructor for ENU and WGS
- Add basic geometric primitives
- Refactor datum and geometry conversions
- Add Square type and raise recordings
- Introduce base Rectangle type support
- Add basic geometric shape structs
- Add WGS and ENU coordinate conversions
- Add structs for common coordinate systems
- Switch to internal Concord API for library declarations
- Enforce Standards for Consistent Representation of Geometric Constants
- Switch to concord fork for dependencies and API
- Generalize vendor API and minimize external dependencies.
- Implement core functionality for converting geographic coordinates
- Init

### <!-- 1 -->🐛 Bug Fixes

- Fix corner order in `get_corners`

### <!-- 2 -->🚜 Refactor

- Refactor: Clean up and improve grid element handling
- Construct Grid object from Polygon
- Use minimum area oriented bounding box calc
- Refactor polygon bounding box calculations
- Cast double values to float explicitly
- Refactor Point and Pose structs
- Refactor Grid initialization with default constructors
- Improve memory safety and performance with `std::span`
- Initialize geometric types in constructors
- Remove redundant Point and coordinate constructors
- Introduce and use Datum struct for coordinates
- "Simplify CMake build and directory structure"

### <!-- 3 -->📚 Documentation

- Improve documentation and refactor IO utilities
- Remove redundant project change log
- Update README to Reflect Changes and corrections
- Introduce revamped Concord++ library with improved documentation and reduced dependencies

### <!-- 4 -->⚡ Performance

- Switch to internal API for ALIAS

### <!-- 5 -->🎨 Styling

- Remove redundant explicit keyword from ctors
- "Refactor internal API and improve style consistency"

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Remove spdlog and add library alias
- Refactor cmake build and examples
- "get_summary_prefix", "parameters": {"prefix": "build"}}: update CMake version and library dependencies
- "Update CMake configuration for dependencies and build on multiple platforms"
- Logo colors
- Logo colors

### Build

- Remove unused CMakeLists.txt_2 file
- Add spdlog dependency when build examples
- Refactor installer to remove example includes
- Improve development environment setup and build system
- Add Nix/Devbox for development environment setup
- "Improve build system and library export with package-config support"

## [0.5.5] - 2025-05-01

### <!-- 0 -->⛰️  Features

- Add constructor to `Point` with ENU and Datum
- Make Datum optional in the Point struct
- Feat: Add Datum field to Point structure
- Refactor geometry types to include Datum
- Add Point constructor for ENU and WGS
- Add basic geometric primitives
- Refactor datum and geometry conversions
- Add Square type and raise recordings
- Introduce base Rectangle type support
- Add basic geometric shape structs
- Add WGS and ENU coordinate conversions
- Add structs for common coordinate systems
- Switch to internal Concord API for library declarations
- Enforce Standards for Consistent Representation of Geometric Constants
- Switch to concord fork for dependencies and API
- Generalize vendor API and minimize external dependencies.
- Implement core functionality for converting geographic coordinates
- Init

### <!-- 2 -->🚜 Refactor

- Refactor Point and Pose structs
- Refactor Grid initialization with default constructors
- Improve memory safety and performance with `std::span`
- Initialize geometric types in constructors
- Remove redundant Point and coordinate constructors
- Introduce and use Datum struct for coordinates
- "Simplify CMake build and directory structure"

### <!-- 3 -->📚 Documentation

- Remove redundant project change log
- Update README to Reflect Changes and corrections
- Introduce revamped Concord++ library with improved documentation and reduced dependencies

### <!-- 5 -->🎨 Styling

- Remove redundant explicit keyword from ctors
- "Refactor internal API and improve style consistency"

### <!-- 7 -->⚙️ Miscellaneous Tasks

- "get_summary_prefix", "parameters": {"prefix": "build"}}: update CMake version and library dependencies
- "Update CMake configuration for dependencies and build on multiple platforms"
- Logo colors
- Logo colors

### Build

- Improve development environment setup and build system
- Add Nix/Devbox for development environment setup
- "Improve build system and library export with package-config support"

<!-- WARP -->
