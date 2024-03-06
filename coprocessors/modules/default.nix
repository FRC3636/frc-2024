{ ... }:

{
  imports = [
    ./networking.nix
    ./administration.nix
    ./services/photonvision.nix
  ];

  config = {
  };
}
