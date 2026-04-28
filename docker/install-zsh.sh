#!/bin/bash
set -e

# Git config (now in user's ~/.gitconfig)
git config --global core.editor "vim"
[ -n "$GIT_NAME" ] && git config --global user.name "$GIT_NAME"
[ -n "$GIT_EMAIL" ] && git config --global user.email "$GIT_EMAIL"

# Oh My Zsh
if [ ! -d "$HOME/.oh-my-zsh" ]; then
    sh -c "$(wget -O- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
fi

ZSH_CUSTOM=${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}
[ ! -d "$ZSH_CUSTOM/plugins/zsh-autosuggestions" ] && \
    git clone https://github.com/zsh-users/zsh-autosuggestions $ZSH_CUSTOM/plugins/zsh-autosuggestions
[ ! -d "$ZSH_CUSTOM/plugins/zsh-syntax-highlighting" ] && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git $ZSH_CUSTOM/plugins/zsh-syntax-highlighting

ZSHRC_FILE="$HOME/.zshrc"
sed -i 's/^plugins=(git)/plugins=(\n    git\n    docker\n    docker-compose\n    zsh-autosuggestions\n    zsh-syntax-highlighting\n)/' "$ZSHRC_FILE"

if ! grep -q "history-beginning-search-backward" "$ZSHRC_FILE"; then
    cat <<'EOT' >> "$ZSHRC_FILE"
bindkey "^[[A" history-beginning-search-backward
bindkey "^[[B" history-beginning-search-forward
EOT
fi
