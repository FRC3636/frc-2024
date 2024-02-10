{ pkgs, ... }:

{
  config = {
    users = {
      mutableUsers = false;

      users.root = {
        hashedPassword = "$y$j9T$Kos3iH12wFUhMpC3EBQ8M.$J.95n6MuTLA5ofsvhwY88TOr8cu2wHhBMz1RzrRdbf2"; # 3636
      };
    };

    services.openssh = {
      enable = true;

      settings = {
        PermitRootLogin = "yes";
      };
    };

    environment.systemPackages = with pkgs; [
      neovim
      htop
    ];
  };
}
