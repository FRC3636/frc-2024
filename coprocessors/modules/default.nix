{ ... }:

{
  imports = [
    ./networking.nix
    ./administration.nix
    ./services/photonvision.nix
  ];

  config = {
    systemd.services.photonvision = {
      serviceConfig = { };
    };
  };
}
