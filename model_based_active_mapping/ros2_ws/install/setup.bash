_mbam_install_setup_dir="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)"
_mbam_workspace_setup="${_mbam_install_setup_dir}/../setup.bash"

if [ -f "$_mbam_workspace_setup" ]; then
  . "$_mbam_workspace_setup"
else
  echo "Workspace setup wrapper not found: ${_mbam_workspace_setup}" 1>&2
  return 1
fi

unset _mbam_workspace_setup
unset _mbam_install_setup_dir
