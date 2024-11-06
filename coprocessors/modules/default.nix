{ ... }:

{
  imports = [
    ./networking.nix
    ./administration.nix
    ./services/photonvision.nix
    ./services/pplib-coprocessor.nix
  ];

  config = {
  };
}
