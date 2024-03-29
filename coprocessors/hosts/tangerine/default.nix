{ ... }:

{
  imports = [
    ./hardware-configuration.nix
  ];

  config = {
    networking = {
      hostName = "tangerine";

      interfaces.enp1s0.wakeOnLan = {
        enable = true;
        policy = [ "phy" ];
      };

      robotNetwork = {
        interface = "enp1s0";
        address = "10.36.36.10";
      };
    };

    services.photonvision.enable = true;

    boot = {
      binfmt.emulatedSystems = [ "aarch64-linux" ];
      loader = {
        systemd-boot.enable = true;
        efi.canTouchEfiVariables = true;
      };
    };

    system.stateVersion = "23.05";
  };
}
