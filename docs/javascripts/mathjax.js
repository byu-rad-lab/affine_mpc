// LaTeX macros for the docs; use these instead of their expansions.
// GitHub's Markdown view does not load this file, so keep them out of README.md.
window.MathJax = {
  tex: {
    macros: {
      bm: ["\\boldsymbol{#1}", 1], // bold vectors and Greek symbols
      T: "\\top", // transpose
      diag: "\\operatorname{diag}",
      R: "\\mathbb{R}",
    },
  },
};
