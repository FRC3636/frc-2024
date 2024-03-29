{ lib
, stdenv
, fetchurl
, makeWrapper
, temurin-jre-bin-11
, bash
, suitesparse
, ...
}:

stdenv.mkDerivation rec {
  pname = "photonvision";
  version = "2024.2.4";

  src = lib.getAttr stdenv.hostPlatform.system {
    "x86_64-linux" = fetchurl {
      url = "https://github.com/PhotonVision/photonvision/releases/download/v${version}/photonvision-v${version}-linuxx64.jar";
      hash = "sha256-97foGMzCFF9ks0RQeHDLzPU1jZuPftYlE0Y8TXl5nr4=";
    };
    "aarch64-linux" = fetchurl {
      url = "https://github.com/PhotonVision/photonvision/releases/download/v${version}/photonvision-v${version}-linuxarm64.jar";
      hash = "sha256-KvQXOHjdvc1RWuN13HYrWONEMexIQxwJXHcJd8RoKGQ=";
    };
  };

  dontUnpack = true;

  nativeBuildInputs = [ makeWrapper ];

  installPhase = ''
    runHook preInstall

    mkdir -p $out/lib
    cp $src $out/lib/photonvision.jar

    makeWrapper ${temurin-jre-bin-11}/bin/java $out/bin/photonvision \
      --prefix LD_LIBRARY_PATH : ${lib.makeLibraryPath [ stdenv.cc.cc.lib suitesparse ]} \
      --prefix PATH : ${lib.makeBinPath [ temurin-jre-bin-11 bash.out ]} \
      --add-flags "-jar $out/lib/photonvision.jar"

    runHook postInstall
  '';

  meta = with lib; {
    description = "The free, fast, and easy-to-use computer vision solution for the FIRST Robotics Competition";
    homepage = "https://photonvision.org/";
    license = licenses.gpl3;
    maintainers = with maintainers; [ max-niederman ];
    platforms = [ "x86_64-linux" "aarch64-linux" ];
  };
}