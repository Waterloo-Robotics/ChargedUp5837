cd "${2:-"./"}"
git pull
git add *
git commit -m "$1"
git push