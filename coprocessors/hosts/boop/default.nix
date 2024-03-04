{ ... }:

{
  config = {
    networking = {
      hostName = "boop";

      robotNetwork = {
        interface = "end1";
        address = "10.36.36.11";
      };
    };

    services.photonvision.enable = true;

    system.stateVersion = "23.05";
  };
}
