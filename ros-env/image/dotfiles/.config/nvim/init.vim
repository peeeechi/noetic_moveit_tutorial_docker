if exists('g:vscode')
    " VSCode extension
    call plug#begin()
    Plug 'tpope/vim-commentary' "コメントプラグイン
    Plug 'tomasiser/vim-code-dark' "vimのカラースキーマ
    Plug 'leafgarland/typescript-vim' "Typescriptのカラースキーマ
    Plug 'rhysd/clever-f.vim'
    Plug 'tpope/vim-surround'
    Plug 'terryma/vim-expand-region'
    call plug#end()
    inoremap <silent> jj <ESC>
    xmap gc  VSCodeCommentary
    nmap gc  VSCodeCommentary
    omap gc  VSCodeCommentary
    nmap gcc VSCodeCommentaryLine

    nnoremap  fp call VSCodeNotify('workbench.action.quickOpen')
    nnoremap  fgs call VSCodeNotify('workbench.view.scm')

    " map v (expand_region_expand)
    " map  (expand_region_shrink)

    set clipboard=unnamed,unnamedplus
else
    call plug#begin()
    " Plug 'junegunn/fzf.vim'
    Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
    Plug 'neoclide/coc.nvim', { 'branch': 'release' }
    Plug 'tpope/vim-fugitive'
    Plug 'tpope/vim-commentary' "コメントプラグイン
    Plug 'tomasiser/vim-code-dark' "vimのカラースキーマ
    Plug 'lambdalisue/fern.vim' "ファイラー
    Plug 'lambdalisue/fern-git-status.vim' "ファイルツリーにgit差分表示
    Plug 'lambdalisue/fern-renderer-nerdfont.vim' "ファイラーのアイコン用2
endif

set number
inoremap <silent> jj <ESC>
