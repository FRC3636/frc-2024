{ lib, fetchFromGitHub, buildDartApplication }:

buildDartApplication rec {
  pname = "pplib-coprocessor";
  version = "v2024.0.0-beta-1";

  src = fetchFromGitHub {
    owner = "mjansen4857";
    repo = "pplib_coprocessor";
    rev = version;
    hash = "sha256-XYlWLvoV+VSuajgjkguBWUhE4brdLNP1oqHSB/6nr5c=";
  };

  dartEntryPoints."pplib_coprocessor" = "bin/pplib_coprocessor.dart";

  pubspecLock = lib.importJSON ./pubspec.lock.json;
  
  meta = with lib; {
    description = "Offload PathPlannerLib's pathfinding to a coprocessor";
    homepage = "https://github.com/mjansen4857/pplib_coprocessor";
    license = licenses.mit;
    maintainers = with maintainers; [ max-niederman ];
  };
}