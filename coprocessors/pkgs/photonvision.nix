{ stdenv, fetchurl, makeWrapper, temurin-jre-bin-11, bash, lib, ... }:

stdenv.mkDerivation rec {
  pname = "photonvision";
  version = "2024.2.3";

  src = {
    "x86_64-linux" = fetchurl {
      url = "https://github.com/PhotonVision/photonvision/releases/download/v${version}/photonvision-v${version}-linuxx64.jar";
      hash = "sha256-45ae9sElAmN6++F9OGAvY/nUl/9UxvHtFxhetKVKfDc=";
    };
    "aarch64-linux" = fetchurl {
      url = "https://github.com/PhotonVision/photonvision/releases/download/v${version}/photonvision-v${version}-linuxarm64.jar";
      hash = "sha256-i/osKO+RAg2nFUPjBdkn3q0Id+uCSTiucfKFVVlEqgs=";
    };
  }."${stdenv.hostPlatform.system}";

  dontUnpack = true;

  nativeBuildInputs = [ makeWrapper ];

  installPhase = ''
    runHook preInstall

    mkdir -p $out/lib
    cp $src $out/lib/photonvision.jar

    makeWrapper ${temurin-jre-bin-11}/bin/java $out/bin/photonvision \
      --prefix LD_LIBRARY_PATH : ${lib.makeLibraryPath [ stdenv.cc.cc.lib ]} \
      --prefix PATH : ${lib.makeBinPath [ temurin-jre-bin-11 bash.out ]} \
      --add-flags "-jar $out/lib/photonvision.jar"

    runHook postInstall
  '';
}