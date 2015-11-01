(setq inhibit-startup-message t)
;;; load-pathを追加する関数を定義
(defun add-to-load-path (&rest paths)
  (let (path)
    (dolist (path paths paths)
     (let ((default-directory (expand-file-name (concat user-emacs-directory path))))
        (add-to-list 'load-path default-directory)
         (if (fboundp 'normal-top-level-add-subdirs-to-load-path)
             (normal-top-level-add-subdirs-to-load-path))))))

;;; ディレクトリをサブディレクトリごとload-pathに追加
(add-to-load-path "elisp" "elpa" "etc" "auto-install")

;;packageの設定
(require 'package)
(add-to-list 'package-archives '("melpa" . "http://melpa.milkbox.net/packages/"))
(package-initialize)

;;auto-installの設定
(require 'auto-install)
(setq auto-install-directory "~/.emacs.d/elisp/")
(auto-install-update-emacswiki-package-name t)
(auto-install-compatibility-setup)

;;;;;;;;;;;;;;;;;;;;;各種設定;;;;;;;;;;;;;;;
;;全体をインデント

;;対応するカッコを表示
(show-paren-mode t)

;;対応するカッコに下線を引く
(setq show-paren-delay 0)
(setq show-paren-style 'expression)
(set-face-attribute 'show-paren-match-face nil
		    :background nil :foreground nil
		    :underline "red" :weight 'extra-bold)

;;らくちんバッファ移動
;(setq windmove-wrap-around t)
;(windmove-default-keybindings)

;;anythingの設定
(require 'anything)
(require 'anything-config)
(add-to-list 'anything-sources 'anything-c-source-emacs-commands)

;;flycheckの設定
(require 'flycheck)
(add-hook 'after-init-hook #'global-flycheck-mode)
(require 'flycheck-pos-tip)
(eval-after-load 'flycheck
  '(custom-set-variables
   '(flycheck-display-errors-function #'flycheck-pos-tip-error-messages)))

;;c-eldocの設定
(require 'c-eldoc)
(add-hook 'c-mode-hook
          (lambda ()
            (set (make-local-variable 'eldoc-idle-delay) 0.20)
            (c-turn-on-eldoc-mode)
            ))
(add-hook 'c++-mode-hook
          (lambda ()
            (set (make-local-variable 'eldoc-idle-delay) 0.20)
            (c-turn-on-eldoc-mode)
            ))

;;auto-completeの設定
(require 'auto-complete-config)
(require 'fuzzy)
(ac-config-default)
(add-to-list 'ac-modes 'text-mode)         ;; text-modeでも自動的に有効にする
(add-to-list 'ac-modes 'fundamental-mode)  ;; fundamental-mode
(add-to-list 'ac-modes 'org-mode)
(add-to-list 'ac-modes 'yatex-mode)
(setq ac-use-menu-map t)       ;; 補完メニュー表示時にC-n/C-pで補完候補選択
(setq ac-use-fuzzy t)          ;; 曖昧マッチ

;;arduino-modeの設定
(load "arduino-mode")
;(add-hook 'arduino-mode '(flycheck-add-next-checker 'avr-gcc))

;;gudの設定
(load "gdb")

;;change-bufferの設定
(load "change-buffer")

;;redo+の設定
(require 'redo+)
(setq undo-no-redo t)
(setq undo-limit 60000)
(setq undo-strong-limit 70000)

;;color-moccurの設定
(require 'color-moccur)
(setq moccur-split-word t)

;;moccur-editの設定
(require 'moccur-edit)

;;;;;;;;;;;;;キーコンフィグ;;;;;;;;;;;;;;;;;;
(global-set-key (kbd "C-:") 'anything)
(global-set-key (kbd "C-c C-i") 'anything-imenu)
(global-set-key [?\C-,] (lambda () (interactive) (my-operate-buffer nil)))
(global-set-key [?\C-.] (lambda () (interactive) (my-operate-buffer t)))
(global-set-key (kbd "C-t") 'indent-whole-buffer)

(define-key gud-minor-mode-map (kbd "<f1>") 'gud-print)
(define-key gud-minor-mode-map (kbd "<S-f1>") 'gud-watch)
(define-key gud-minor-mode-map (kbd "<f2>") 'gud-refresh)
(define-key gud-minor-mode-map (kbd "<f5>") 'gud-cont)
(define-key gud-minor-mode-map (kbd "<S-f5>") 'gud-kill)
(define-key gud-minor-mode-map (kbd "<f6>") 'gud-until)
(define-key gud-minor-mode-map (kbd "<f9>") 'gdb-set-clear-breakpoint)
(define-key gud-minor-mode-map (kbd "<S-f9>") 'gud-break-main)
(define-key gud-minor-mode-map (kbd "<f10>") 'gud-next)
(define-key gud-minor-mode-map (kbd "<f11>") 'gud-step)
(define-key gud-minor-mode-map (kbd "<C-f10>") 'gud-until)
(define-key gud-minor-mode-map (kbd "<C-f11>") 'gud-finish)
(define-key gud-minor-mode-map (kbd "<S-f11>") 'gud-finish)
