<img align="right" width="26%" src="./misc/logo.png">

# Concord

Coordinate transformations (WGS/ECF/UTM/ENU/NED) with `datapod` as the mandatory backend.

## What Concord Owns

- `concord::earth::WGS`, `concord::earth::ECF`, `concord::earth::UTM`
- `concord::frame::ENU`, `concord::frame::NED`, `concord::frame::Datum`
- Conversion functions + fluent `concord::convert()` builder

Everything else (math/geometry/containers/error handling) comes from `datapod` (`dp::Point`, `dp::Result`, `dp::Error`, ...).

## Build (repo)

```bash
make config
make build
make test
```

## Use

```cpp
#include <concord/concord.hpp>
using namespace concord;

earth::WGS paris{48.8566, 2.3522, 35.0};
earth::ECF ecf = earth::to_ecf(paris);

auto utm = earth::to_utm(paris);      // dp::Result<earth::UTM>
auto wgs = earth::to_wgs(utm.value()); // dp::Result<earth::WGS>

frame::Datum datum{paris};
frame::ENU enu = frame::to_enu(datum, paris);

auto roundtrip = convert(paris)
  .withDatum(datum)
  .to<frame::ENU>()
  .to<earth::WGS>()
  .build(); // dp::Result<earth::WGS>
```

## Examples

- `examples/wgs_to_enu_example.cpp`
- `examples/wgs_to_utm_example.cpp`
- `examples/convert_builder_example.cpp`

