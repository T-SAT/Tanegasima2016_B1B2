(require 'gud)

(setq gdb-many-windows t)
(setq gdb-use-separate-io-buffer t)
(add-hook
 'gdb-mode-hook
 '(lambda ()
    (gud-tooltip-mode t)
    (gud-def gud-break-main "break main" nil "Set breakpoint at main.")
 ))
(setq gud-tooltip-echo-area nil)

(defun gdb-set-clear-breakpoint ()
  (interactive)
  (if (or (buffer-file-name) (eq major-mode 'gdb-assembler-mode))
      (if (or
           (let ((start (- (line-beginning-position) 1))
                 (end (+ (line-end-position) 1)))
             (catch 'breakpoint
               (dolist (overlay (overlays-in start end))
                 (if (overlay-get overlay 'put-break)
                     (throw 'breakpoint t)))))
           (eq (car (fringe-bitmaps-at-pos)) 'breakpoint))
          (gud-remove nil)
        (gud-break nil))))

(defun gud-kill ()
  "Kill gdb process."
  (interactive)
  (with-current-buffer gud-comint-buffer (comint-skip-input))
  (kill-process (get-buffer-process gud-comint-buffer)))
