{ lib, stdenv, cmake, fetchFromGitHub, python3, blas, lapack, nlohmann_json, eigen, openblas }:

stdenv.mkDerivation rec {
  pname = "acados";
  version = "0.3.3";

  src = fetchFromGitHub {
    owner = "acados";
    repo = "acados";
    rev = "v0.3.3";
    hash = "sha256-YauG763mvtcLmvAD3Q11KJnv48XuotxMO5FZ509UjmM=";
    fetchSubmodules = true;
  };

  nativeBuildInputs = [ cmake ];
  buildInputs = [
    python3
    blas
    lapack
    nlohmann_json
    eigen
    openblas
    python3.pkgs.numpy
    python3.pkgs.setuptools
  ];

  cmakeFlags = [
    "-DACADOS_WITH_QPOASES=ON"
    "-DACADOS_EXAMPLES=ON"
    "-DHPIPM_TARGET=GENERIC"
    "-DBLASFEO_TARGET=GENERIC"
    "-DACADOS_INSTALL_DIR=${placeholder "out"}"
  ];

  env.NIX_CFLAGS_COMPILE = lib.concatStringsSep " " [
    "-Wno-error=implicit-function-declaration"
    "-Wno-error=incompatible-pointer-types"
    "-Wno-error=int-conversion"
  ];

  installPhase = ''
    mkdir -p $out
    make install
    find $out/lib -name "*.so" -exec patchelf --set-rpath "$(patchelf --print-rpath {}):${lib.makeLibraryPath buildInputs}" {} \;
  '';

  meta = with lib; {
    description = "A modular open-source framework for fast embedded optimal control";
    homepage = "https://github.com/acados/acados";
    license = licenses.asl20;
    platforms = platforms.all;
  };
}
