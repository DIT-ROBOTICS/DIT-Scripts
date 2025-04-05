#!/bin/bash -e

# ========== Prompt for DRY_RUN ==========
read -p "Enable dry-run mode? (y/N): " yn
case "$yn" in
    [Yy]* ) export DRY_RUN=true ;;
    * ) export DRY_RUN=false ;;
esac

# ========== Config ==========
export USE_GITHUB_API=true

# ========== Color codes ==========
export NC='\033[0m'
export GREEN='\033[0;32m'
export BLUE='\033[0;34m'
export YELLOW='\033[1;33m'
export RED='\033[0;31m'

# ========== Header ==========
printf "${BLUE}========== Submodule Update Started ==========\n"
printf "Config → USE_GITHUB_API=%s, DRY_RUN=%s\n" "$USE_GITHUB_API" "$DRY_RUN"
printf "==============================================${NC}\n\n"

# ========== Main ==========
git submodule foreach --quiet --recursive '
  bash -e -c "
    name=\"\$1\"
    path=\"\$2\"
    toplevel=\"\$3\"

    printf \"──────────────────────────────────────────────\n\"
    printf \"[Submodule]    %s\n\" \"\$name\"
    printf \"Path        →  %s\n\" \"\$path\"

    get_latest_github_release_tag() {
      local remote_url=\$1
      local repo_path
      repo_path=\$(echo \"\$remote_url\" | sed -E \"s|.*github\\.com[/:]([^/]+/[^/.]+)(\\.git)?|\1|\")
      local api_url=\"https://api.github.com/repos/\${repo_path}/releases/latest\"
      local tag
      tag=\$(curl -s \"\$api_url\" | jq -r .tag_name)
      echo \"\$tag\"
    }

    get_latest_tag_by_git() {
      git fetch --tags > /dev/null
      git tag -l | sort -V | tail -n 1
    }

    cd \"\$toplevel/\$path\" || exit 1
    remote_url=\$(git remote get-url origin)
    printf \"Remote      →  %s\n\" \"\$remote_url\"

    if [ \"$USE_GITHUB_API\" = true ]; then
      latest_tag=\$(get_latest_github_release_tag \"\$remote_url\")
    fi

    if [ -z \"\$latest_tag\" ] || [ \"\$latest_tag\" = \"null\" ]; then
      latest_tag=\$(get_latest_tag_by_git)
      printf \"${YELLOW}Note        →  GitHub API fallback to git tags.${NC}\n\"
    fi

    if [ -z \"\$latest_tag\" ]; then
      printf \"${YELLOW}Result      →  No valid tags found. Skipping.${NC}\n\"
      exit 0
    fi

    current_tag=\$(git describe --tags --exact-match 2>/dev/null || echo \"none\")
    printf \"Current tag →  %s\n\" \"\$current_tag\"
    printf \"Latest tag  →  %s\n\" \"\$latest_tag\"

    if [ \"\$current_tag\" = \"\$latest_tag\" ]; then
      printf \"${GREEN}Result      →  Already up to date.${NC}\n\"
    else
      if [ \"$DRY_RUN\" = true ]; then
        printf \"${YELLOW}Result      →  Would update from %s to %s (dry-run only).${NC}\n\" \"\$current_tag\" \"\$latest_tag\"
      else
        printf \"${BLUE}Action      →  Updating to %s...${NC}\n\" \"\$latest_tag\"
        git fetch --tags
        git checkout \"\$latest_tag\" > /dev/null 2>&1

        cd \"\$toplevel\" || exit 1
        git add \"\$path\"
        git commit -m \"Update submodule \$name to \$latest_tag\" > /dev/null
        printf \"${GREEN}Result      →  Updated and committed to %s${NC}\n\" \"\$latest_tag\"
      fi
    fi
  " _ "$name" "$path" "$toplevel"
'

printf "\n${BLUE}========= Submodule Update Completed =========${NC}\n"
